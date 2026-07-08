//! The Rubik's cube scene: the full real-world pipeline. This is yours now.
//!
//! A self-contained copy of the real-world cube pipeline: metric scale
//! (a 4.5cm Rubik's cube, camera orbiting at 0.1-0.25m), joint depth+color
//! observations, illuminant tint, and the two-scale inference kernel from the
//! live demo. Everything is in this file. Edit the model, the priors, the
//! moves, and rerun.
//!
//! Run it (synthetic, no hardware needed):
//!   cargo run --release --example sandbox -- rubiks
//!
//! Got an Intel RealSense D435? See the `run_rgbd_loop` block at the bottom
//! of run(). The identical model and kernel work with real camera frames
//!
//! ─── EXTENSION IDEAS ─────────────────────────────────────────────────────────
//!
//! - Second cube: sample another (u, v), push another `Cube` into the scene.
//!   Watch for multimodality when one occludes the other. This is where a
//!   single MH chain starts to struggle, and the motivation for smarter moves.
//! - Swap the cube for a `Cylinder` (a mug): `core::solid::Cylinder` already
//!   implements `Solid`. Which latents does it need? What identifies its
//!   rotation... anything?
//! - Make the cube size a latent again (`uniform(0.01, 0.05)`): at close
//!   range, can depth still separate size from distance? (This is the classic
//!   scale/depth ambiguity -- the fixed size below is how the live demo
//!   sidesteps it.)
//! - Sharpen or break the illuminant prior (`illum_c*`): what happens on a
//!   strongly tinted "observation" if the prior can't express the tint?
//! - Replace a regen move with a smarter proposal: the pose-net crate trains
//!   a CNN on samples from *this model* and uses it as an MH proposal (`pose_guide`).

#![allow(non_upper_case_globals)]

use std::f32::consts::PI;

use glam::{Affine3A, EulerRot, Mat4, Quat, Vec3, Vec3A};
use modppl::prelude::*;
use modppl_derender::{
    config::{apply_cube_pipeline_defaults, AREA, FAR, FOVY, H, NEAR, W},
    core::*,
    image::{rgb, Colors, Depths},
    inference::InferenceKernel,
    sandbox::{run_sandbox_loop, Panels},
};

const DEPTH_NOISE: f32 = 0.1;
const COLOR_NOISE: f32 = 0.1;

/// probability of a full prior resample (vs. a local drift step) for the
/// orbit latents: mostly drift for smooth refinement, occasionally a global
/// jump so the chain can escape a bad lock.
const ORBIT_RESAMPLE_PROB: f32 = 0.1;

// ─── the orbital camera ──────────────────────────────────────────────────────
//
// Camera on a hemisphere around `target`, always looking at it (plus a small
// jitter). Two modeling insights baked in:
//
// - `sin_elevation` (not the elevation angle) is the latent: the hemisphere
//   area element is cos(e) de dφ, so uniform elevation oversamples the pole;
//   sin(e) ~ Uniform(0,1) gives uniform solid-angle coverage.
// - the look-at jitter decouples camera orientation from the exact target
//   hypothesis: with an exact look-at, a tiny cube-position proposal swings
//   the whole rendered frame, making small changes score terribly.

fn orbit_camera(
    azimuth: f32,
    sin_elevation: f32,
    radius: f32,
    target: Vec3A,
    jitter_yaw: f32,
    jitter_pitch: f32,
) -> Affine3A {
    let cos_elevation = (1.0 - sin_elevation * sin_elevation).sqrt();
    let height = radius * sin_elevation;
    let horiz = radius * cos_elevation;
    let eye: Vec3 =
        (target + Vec3A::new(horiz * azimuth.cos(), height, horiz * azimuth.sin())).into();
    let view = Affine3A::look_at_rh(eye, target.into(), Vec3::Y);
    let jitter = Affine3A::from_quat(Quat::from_euler(
        EulerRot::YXZ,
        jitter_yaw,
        jitter_pitch,
        0.0,
    ));
    view.inverse() * jitter
}

// ─── the model: the real-world cube pipeline, in one place ──────────────────

dyngen!(
    fn uniform_color(lb: f32, ub: f32) -> [f32; 3] {
        [
            uniform(lb, ub) %= "c0",
            uniform(lb, ub) %= "c1",
            uniform(lb, ub) %= "c2",
        ]
    }
);

dyngen!(
    fn rubiks_model(noise: (f32, f32, bool)) -> (Depths, Colors) {
        let (depth_noise, color_noise, path_trace) = noise;

        // a real Rubik's cube is ~4.5cm wide; with size known, orbit_radius is
        // the only scale parameter, so depth identifies it cleanly.
        const CUBE_HALF_EXTENT: f32 = 0.0225;

        let u = uniform(-1.0, 1.0) %= "cube_u";
        let v = uniform(-1.0, 0.0) %= "cube_v";
        let cube = (
            Box::new(Cube {
                center: [u, CUBE_HALF_EXTENT, v].into(),
                half_extent: CUBE_HALF_EXTENT,
            }) as Box<dyn Solid>,
            [0.0, 0.0, 0.0], // ignored: Cube::color_at overrides per-face
        );

        // camera: uniform over the hemisphere above the cube, close range
        let orbit_azimuth = uniform(0.0, 2.0 * PI) %= "orbit_azimuth";
        let orbit_sin_elevation = uniform(0.0, 1.0) %= "orbit_sin_elevation";
        let orbit_radius = uniform(0.1, 0.25) %= "orbit_radius";
        let lookat_yaw_jitter = normal(0.0, 0.05) %= "lookat_yaw_jitter";
        let lookat_pitch_jitter = normal(0.0, 0.05) %= "lookat_pitch_jitter";
        let x = orbit_camera(
            orbit_azimuth,
            orbit_sin_elevation,
            orbit_radius,
            [u, CUBE_HALF_EXTENT, v].into(),
            lookat_yaw_jitter,
            lookat_pitch_jitter,
        );

        // unknown ground color (c0=B, c1=G, c2=R -- see `image::rgb` if you want
        // to hardcode a color literal instead, since a bare [r, g, b]-looking
        // array is quietly BGR here). An infinite plane dominates a close-up
        // tabletop frame, so a fixed guess would swamp the color likelihood with
        // error.
        let ground_color = uniform_color(0.0, 1.0) /= "ground";
        let ground = (
            Box::new(Plane {
                origin: Vec3A::ZERO,
                normal: [0.0, 1.0, 0.0].into(),
            }) as Box<dyn Solid>,
            ground_color
        );

        let scene = vec![ground, cube];
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut depths = vec![0.0; AREA()];
        let mut colors = vec![[0.0; 3]; AREA()];
        if path_trace {
            raytrace_depths_and_path_colors(
                x,
                proj,
                &scene,
                rgb(0.7, 0.7, 0.7),
                &mut depths,
                &mut colors,
            );
        } else {
            raytrace_depths_and_flat_colors(
                x,
                proj,
                &scene,
                rgb(0.7, 0.7, 0.7),
                &mut depths,
                &mut colors,
            );
        }

        // illuminant/white-balance tint: one multiplicative latent absorbing most
        // of the real camera's color cast.
        let illum = uniform_color(0.5, 1.0) /= "illum";
        for c in colors.iter_mut() {
            c[0] *= illum[0];
            c[1] *= illum[1];
            c[2] *= illum[2];
        }

        noisy_depths(depths.clone(), depth_noise) %= "depth_observation";
        noisy_colors(colors.clone(), color_noise) %= "color_observation";

        (depths, colors)
    }
);

dyngen!(
    fn drift(
        trace: Weak<DynTrace<(f32, f32, bool), (Depths, Colors)>>,
        pass: AddrMap,
        stdev: f32,
    ) {
        let trace = trace.upgrade().unwrap();
        for addr in &addrs_of(&pass) {
            normal(trace.data.read::<f32>(addr), stdev) %= addr.as_str();
        }
    }
);

// ─── inference ───────────────────────────────────────────────────────────────

pub fn run() {
    // canonical cube-pipeline env (RES/FOV/NEAR/FAR); env vars still override
    apply_cube_pipeline_defaults();

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

    let mut env_pass = AddrMap::new();
    env_pass.visit("ground");
    env_pass.visit("illum");

    run_sandbox_loop(
        "sandbox: rubiks  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            // the orbital camera always points at the cube, so an
            // unconstrained prior sample always has it in frame
            let gt = rubiks_model
                .generate((DEPTH_NOISE, COLOR_NOISE, false), DynTrie::new())
                .0;
            let obs_depth = gt.data.read::<Depths>("depth_observation").clone();
            let obs_color = gt.data.read::<Colors>("color_observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("depth_observation", Arc::new(obs_depth.clone()));
            constraints.observe("color_observation", Arc::new(obs_color.clone()));
            let trace = rubiks_model
                .generate((DEPTH_NOISE, COLOR_NOISE, false), constraints)
                .0;

            // the live demo's kernel: drift for smooth refinement, an
            // occasional prior resample for recovery, plain regen for the
            // strongly-identified environment latents.
            let kernel = InferenceKernel::new(&rubiks_model)
                .regen_mh(&cube_pass)
                .mh(&drift, (pass_of(&["cube_u", "cube_v"]), 0.05))
                .then(|t| {
                    let mut rng = ThreadRng::default();
                    if u01(&mut rng) < ORBIT_RESAMPLE_PROB {
                        regen_mh(&rubiks_model, t, &orbit_azimuth_pass)
                    } else {
                        mh(&rubiks_model, t, &drift, (pass_of(&["orbit_azimuth"]), 0.1))
                    }
                })
                .then(|t| {
                    let mut rng = ThreadRng::default();
                    if u01(&mut rng) < ORBIT_RESAMPLE_PROB {
                        regen_mh(&rubiks_model, t, &orbit_radius_elevation_pass)
                    } else {
                        mh(
                            &rubiks_model,
                            t,
                            &drift,
                            (pass_of(&["orbit_radius", "orbit_sin_elevation"]), 0.05),
                        )
                    }
                })
                .regen_mh(&lookat_jitter_pass)
                .regen_mh(&env_pass);

            let mut trace = trace;
            move |_keys: &[minifb::Key]| {
                trace = kernel.step(trace.clone());
                let (hyp_depth, hyp_color) = trace.retv.clone().unwrap();
                Panels::Rgbd {
                    obs: (obs_depth.clone(), obs_color.clone()),
                    hyp: (hyp_depth, hyp_color),
                }
            }
        },
    )
    .expect("window error");

    // ─── RealSense variant ───────────────────────────────────────────────────
    //
    // The same model and the same kernel, driven by real camera frames: swap
    // the harness. Replace the run_sandbox_loop call above with (uncomment,
    // and add `live::run_rgbd_loop` to the imports):
    //
    // let mut trace: Option<DynTrace<(f32, f32, bool), (Depths, Colors)>> = None;
    // run_rgbd_loop(|obs_depth, obs_color| {
    //     let mut constraints = DynTrie::new();
    //     constraints.observe("depth_observation", Arc::new(obs_depth));
    //     constraints.observe("color_observation", Arc::new(obs_color));
    //     // warm-start from the previous frame's posterior instead of
    //     // restarting -- this is what makes it *tracking*, not per-frame
    //     // inference from scratch
    //     let current = match trace.take() {
    //         None => rubiks_model.generate((DEPTH_NOISE, COLOR_NOISE, false), constraints).0,
    //         Some(t) => rubiks_model.update(
    //             t, (DEPTH_NOISE, COLOR_NOISE, false), ArgDiff::NoChange, constraints,
    //         ).0,
    //     };
    //     let result = InferenceKernel::new(&rubiks_model)
    //         /* ...the same moves as above... */
    //         .iter(current).take(5).last().unwrap();
    //     let hyp = result.retv.clone().unwrap();
    //     trace = Some(result);
    //     hyp
    // }).expect("camera error");
}
