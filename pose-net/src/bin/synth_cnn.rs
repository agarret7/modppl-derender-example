//! Synthetic test of the CNN-guided MH proposal -- no camera, no sim-to-real
//! gap: the observation is sampled from `cube_rgbd_model` itself, i.e. exactly
//! the distribution the PoseNet trained on.
//!
//! On each (re)start it prints the ground-truth orbit pose next to the CNN's
//! single-shot estimate, then runs windowed MCMC (one sweep per display frame)
//! on the sandbox harness, so convergence speed is visible.
//!
//! Keys (harness: Space=pause, R=resample+restart, S=snapshot, ESC=quit) plus:
//!   C — toggle the CNN-guided move on/off (watch convergence speed change)
//!
//!   cargo run --release -p pose-net --bin synth_cnn [--weights out/pose_net.safetensors]

use std::cell::Cell;

use minifb::Key;
use modppl::prelude::*;
use modppl_derender::{
    core::*,
    config::{W, H},
    image::{Colors, Depths},
    inference::Kernel,
    sandbox::{run_sandbox_loop, Panels},
};
use pose_net::PoseEstimator;

const DEPTH_NOISE: f32 = 0.1;
const COLOR_NOISE: f32 = 0.1;
const MIN_AZIMUTH_CONFIDENCE: f64 = 0.25;
const ORBIT_RESAMPLE_PROB: f64 = 0.1;

fn main() -> anyhow::Result<()> {
    modppl_derender::config::apply_cube_pipeline_defaults();
    let weights = std::env::args().skip(1)
        .skip_while(|a| a != "--weights").nth(1)
        .unwrap_or_else(|| "out/pose_net.safetensors".to_string());

    let estimator = PoseEstimator::load(&weights, H(), W())?;
    println!("loaded PoseNet weights from {weights} ({}x{})", W(), H());

    let mut cube_mask = AddrMap::new();
    cube_mask.visit("cube_u");
    cube_mask.visit("cube_v");

    let mut orbit_azimuth_mask = AddrMap::new();
    orbit_azimuth_mask.visit("orbit_azimuth");

    let mut orbit_radius_elevation_mask = AddrMap::new();
    orbit_radius_elevation_mask.visit("orbit_radius");
    orbit_radius_elevation_mask.visit("orbit_sin_elevation");

    let mut lookat_jitter_mask = AddrMap::new();
    lookat_jitter_mask.visit("lookat_yaw_jitter");
    lookat_jitter_mask.visit("lookat_pitch_jitter");

    let mut ground_mask = AddrMap::new();
    ground_mask.visit("ground_c0");
    ground_mask.visit("ground_c1");
    ground_mask.visit("ground_c2");
    ground_mask.visit("illum_c0");
    ground_mask.visit("illum_c1");
    ground_mask.visit("illum_c2");

    // shared across restarts; the C key toggles it, the kernel's move reads it
    let cnn_enabled = Cell::new(true);

    run_sandbox_loop(
        "synthetic CNN-guided derender  |  C=toggle CNN  Space=pause  R=resample  S=save  ESC=quit",
        || {
            // ground truth from the prior; observation = its noisy render
            let gt = cube_rgbd_model.generate((DEPTH_NOISE, COLOR_NOISE, false), DynTrie::new()).0;
            let obs_depth = gt.data.read::<Depths>("depth_observation").clone();
            let obs_color = gt.data.read::<Colors>("color_observation").clone();

            let gt_az   = gt.data.read::<f64>("orbit_azimuth");
            let gt_elev = gt.data.read::<f64>("orbit_sin_elevation");
            let gt_rad  = gt.data.read::<f64>("orbit_radius");

            let est = estimator.estimate(&obs_depth, &obs_color).expect("CNN forward failed");
            let az_err = {
                let d = (est.azimuth - gt_az).abs();
                d.min(std::f64::consts::TAU - d).to_degrees()
            };
            println!("---");
            println!("truth: az {gt_az:.2}  sin_elev {gt_elev:.2}  radius {gt_rad:.2}");
            println!(
                "cnn:   az {:.2} (err {az_err:.0} deg, conf {:.2})  sin_elev {:.2} (err {:.2})  radius {:.2} (err {:.2})",
                est.azimuth, est.azimuth_confidence,
                est.sin_elevation, (est.sin_elevation - gt_elev).abs(),
                est.radius, (est.radius - gt_rad).abs(),
            );

            let mut constraints = DynTrie::new();
            constraints.observe("depth_observation", Arc::new(obs_depth.clone()));
            constraints.observe("color_observation", Arc::new(obs_color.clone()));
            let trace = cube_rgbd_model.generate((DEPTH_NOISE, COLOR_NOISE, false), constraints).0;

            let cnn_accepts = Cell::new(0u32);
            let cnn_attempts = Cell::new(0u32);
            let cnn_enabled = &cnn_enabled;

            let mut kernel = Kernel::new(&cube_rgbd_model, trace)
                .regen_mh(&cube_mask)
                .mh(&rgbd_drift, (vec!["cube_u", "cube_v"], 0.05))
                .then(move |t| {
                    if cnn_enabled.get() && est.azimuth_confidence > MIN_AZIMUTH_CONFIDENCE {
                        cnn_attempts.set(cnn_attempts.get() + 1);
                        let (t, accepted) = mh(&cube_rgbd_model, t, &pose_guide, vec![
                            ("orbit_azimuth", est.azimuth, 0.3),
                            ("orbit_sin_elevation", est.sin_elevation.clamp(0.05, 0.95), 0.1),
                            ("orbit_radius", est.radius.clamp(0.11, 0.24), 0.05),
                        ]);
                        if accepted { cnn_accepts.set(cnn_accepts.get() + 1); }
                        if cnn_attempts.get() % 60 == 0 {
                            println!("cnn moves: {}/{} accepted", cnn_accepts.get(), cnn_attempts.get());
                        }
                        (t, accepted)
                    } else {
                        (t, false)
                    }
                })
                .then(|t| {
                    let mut rng = ThreadRng::default();
                    if u01(&mut rng) < ORBIT_RESAMPLE_PROB {
                        regen_mh(&cube_rgbd_model, t, &orbit_azimuth_mask)
                    } else {
                        mh(&cube_rgbd_model, t, &rgbd_drift, (vec!["orbit_azimuth"], 0.1))
                    }
                })
                .then(|t| {
                    let mut rng = ThreadRng::default();
                    if u01(&mut rng) < ORBIT_RESAMPLE_PROB {
                        regen_mh(&cube_rgbd_model, t, &orbit_radius_elevation_mask)
                    } else {
                        mh(&cube_rgbd_model, t, &rgbd_drift, (vec!["orbit_radius", "orbit_sin_elevation"], 0.05))
                    }
                })
                .regen_mh(&lookat_jitter_mask)
                .regen_mh(&ground_mask);

            move |keys: &[Key]| {
                if keys.contains(&Key::C) {
                    cnn_enabled.set(!cnn_enabled.get());
                    println!("cnn moves: {}", if cnn_enabled.get() { "ON" } else { "OFF" });
                }
                let t = kernel.next().unwrap();
                Panels::Rgbd {
                    obs: (obs_depth.clone(), obs_color.clone()),
                    hyp: t.retv.clone().unwrap(),
                }
            }
        },
    )
}
