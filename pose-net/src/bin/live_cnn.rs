//! Live RGB-D cube derender with a CNN-guided MH proposal.
//!
//! Same model and MCMC structure as `live_rgbd_cube`, plus one extra move per
//! sweep: an independence proposal centered on a trained PoseNet's estimate of
//! the orbit pose (see `pose_guide`). The MH acceptance test corrects for CNN
//! error, so a wrong estimate costs a rejected proposal, never a wrong answer.
//!
//! The CNN was trained at a fixed resolution (see the dataset's meta.txt) --
//! run with RES matching it:
//!
//!   RES=128 NEAR_M=0.15 FAR_M=1.0 cargo run --release -p pose-net --bin live_cnn

use modppl::prelude::*;
use modppl_derender::{
    config::{H, W},
    core::*,
    image::{Colors, Depths},
    inference::InferenceKernel,
    live::run_rgbd_loop,
};
use pose_net::PoseEstimator;

const DEFAULT_DEPTH_NOISE: f32 = 0.2;
const DEFAULT_COLOR_NOISE: f32 = 0.32;
const SWEEPS_PER_FRAME: usize = 5;
const ORBIT_RESAMPLE_PROB: f32 = 0.1;

/// below this azimuth-head norm the view is ambiguous (near-pole) and the
/// CNN's azimuth is noise -- skip the guided move rather than spam rejections.
const MIN_AZIMUTH_CONFIDENCE: f32 = 0.25;

fn parse_args() -> (f32, f32, String) {
    let args: Vec<String> = std::env::args().collect();
    let mut depth_noise = DEFAULT_DEPTH_NOISE;
    let mut color_noise = DEFAULT_COLOR_NOISE;
    let mut weights = "out/pose_net.safetensors".to_string();
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--d-noise" | "-d" => {
                if let Some(v) = args.get(i + 1).and_then(|s| s.parse().ok()) {
                    depth_noise = v;
                    i += 2;
                } else {
                    eprintln!("--d-noise requires a float");
                    i += 1;
                }
            }
            "--rgb-noise" | "-c" => {
                if let Some(v) = args.get(i + 1).and_then(|s| s.parse().ok()) {
                    color_noise = v;
                    i += 2;
                } else {
                    eprintln!("--rgb-noise requires a float");
                    i += 1;
                }
            }
            "--weights" | "-w" => {
                if let Some(v) = args.get(i + 1) {
                    weights = v.clone();
                    i += 2;
                } else {
                    eprintln!("--weights requires a path");
                    i += 1;
                }
            }
            _ => {
                i += 1;
            }
        }
    }
    (depth_noise, color_noise, weights)
}

fn main() -> anyhow::Result<()> {
    modppl_derender::config::apply_cube_pipeline_defaults();
    let (depth_noise, color_noise, weights) = parse_args();
    println!("noise: d={depth_noise}  rgb={color_noise}");

    let estimator = PoseEstimator::load(&weights, H(), W())?;
    println!("loaded PoseNet weights from {weights} ({}x{})", W(), H());

    let mut cube_pass = AddrMap::new();
    cube_pass.visit("cube_u");
    cube_pass.visit("cube_v");

    let mut orbit_azimuth_pass = AddrMap::new();
    orbit_azimuth_pass.visit("orbit_azimuth");

    let mut orbit_radius_elevation_pass = AddrMap::new();
    orbit_radius_elevation_pass.visit("orbit_radius");
    orbit_radius_elevation_pass.visit("orbit_sin_elevation");

    let mut lookat_jitter_pass = AddrMap::new();
    lookat_jitter_pass.visit("lookat_yaw_jitter");
    lookat_jitter_pass.visit("lookat_pitch_jitter");

    let mut ground_albedo_pass = AddrMap::new();
    ground_albedo_pass.visit("ground_c0");
    ground_albedo_pass.visit("ground_c1");
    ground_albedo_pass.visit("ground_c2");
    ground_albedo_pass.visit("illum_c0");
    ground_albedo_pass.visit("illum_c1");
    ground_albedo_pass.visit("illum_c2");

    let mut trace: Option<DynTrace<(f32, f32, bool), (Depths, Colors)>> = None;

    run_rgbd_loop(|obs_depth, obs_color| {
        // one CNN forward pass per frame, on the raw observation
        let est = estimator.estimate(&obs_depth, &obs_color).ok();
        if let Some(e) = &est {
            println!(
                "cnn: az {:.2} rad (conf {:.2})  sin_elev {:.2}  radius {:.2} m",
                e.azimuth, e.azimuth_confidence, e.sin_elevation, e.radius
            );
        }

        let mut constraints = DynTrie::new();
        constraints.observe("depth_observation", Arc::new(obs_depth));
        constraints.observe("color_observation", Arc::new(obs_color));

        let current = match trace.take() {
            None => {
                cube_rgbd_model
                    .generate((depth_noise, color_noise, false), constraints)
                    .0
            }
            Some(t) => {
                cube_rgbd_model
                    .update(
                        t,
                        (depth_noise, color_noise, false),
                        ArgDiff::NoChange,
                        constraints,
                    )
                    .0
            }
        };

        let result = InferenceKernel::new(&cube_rgbd_model)
            .regen_mh(&cube_pass)
            .mh(&rgbd_drift, (pass_of(&["cube_u", "cube_v"]), 0.05))
            // CNN-guided independence move: proposes the full orbit pose at
            // once, centered on the estimate. Stdevs reflect typical CNN
            // error; centers clamped inside the priors' support so proposals
            // aren't wasted on automatic -inf rejections.
            .then(|t| match &est {
                Some(e) if e.azimuth_confidence > MIN_AZIMUTH_CONFIDENCE => mh(
                    &cube_rgbd_model,
                    t,
                    &pose_guide,
                    vec![
                        ("orbit_azimuth", e.azimuth, 0.3),
                        (
                            "orbit_sin_elevation",
                            e.sin_elevation.clamp(0.05, 0.95),
                            0.1,
                        ),
                        ("orbit_radius", e.radius.clamp(0.11, 0.24), 0.05),
                    ],
                ),
                _ => (t, false),
            })
            .then(|t| {
                let mut rng = ThreadRng::default();
                if u01(&mut rng) < ORBIT_RESAMPLE_PROB {
                    regen_mh(&cube_rgbd_model, t, &orbit_azimuth_pass)
                } else {
                    mh(
                        &cube_rgbd_model,
                        t,
                        &rgbd_drift,
                        (pass_of(&["orbit_azimuth"]), 0.1),
                    )
                }
            })
            .then(|t| {
                let mut rng = ThreadRng::default();
                if u01(&mut rng) < ORBIT_RESAMPLE_PROB {
                    regen_mh(&cube_rgbd_model, t, &orbit_radius_elevation_pass)
                } else {
                    mh(
                        &cube_rgbd_model,
                        t,
                        &rgbd_drift,
                        (pass_of(&["orbit_radius", "orbit_sin_elevation"]), 0.05),
                    )
                }
            })
            .regen_mh(&lookat_jitter_pass)
            .regen_mh(&ground_albedo_pass)
            .iter(current)
            .take(SWEEPS_PER_FRAME)
            .last()
            .unwrap();

        let hyp = result.retv.clone().unwrap();
        trace = Some(result);
        hyp
    })
}
