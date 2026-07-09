//! Tutorial 02: a scene is a probabilistic program.
//!
//! Same pattern as tutorial_01 (simulate, condition, regenerate choices),
//! just with a whole image standing in for a single number. The simplest
//! possible derenderer: infer the camera's height and roll from a depth
//! image of an infinite ground plane.
//!
//! Run it:
//!   cargo run --release --example tutorial_02_ground
//!
//! Window: observed depth (left) vs. the chain's current hypothesis (right).
//! Keys: Space = pause, R = sample a fresh scene and restart, S = snapshot,
//! ESC = quit. Pressing R repeatedly *is* prior visualization. each left
//! panel is one draw from the model.

#![allow(non_upper_case_globals)]

use std::f32::consts::PI;

use glam::{Affine3A, EulerRot, Mat4, Quat, Vec3A};
use modppl::prelude::*;
use modppl_derender::{
    config::{AREA, FAR, FOVY, H, NEAR, W},
    core::*,
    image::Depths,
    inference::InferenceKernel,
    print_trace_live,
    sandbox::{run_sandbox_loop, Panels},
};

/// Observation noise. EXERCISE: raise this to 0.3 and watch
/// the posterior get more uncertain. The scene parameters have wider variance,
/// because more poses explain the noisy image about equally well.
const NOISE: f32 = 0.1;

// ─── the scene model ──────────────────────────────────────────────────────

// A likelihood in ModPPL is simple: any custom type implementing
// `Distribution<Value, Params>` is a sampleable base distribution (%=)
// that implements two functions: Distribution::logpdf and Distribution::random
// Read more about them: github.com/agarret7/modppl/tree/main/modppl/src/modeling/dists

// This one scores an observed depth image against a rendered one,
// pixel by pixel, under truncated-Gaussian pixel noise (truncated to [0,1],
// the renderer's normalized depth range).
use modppl_derender::noisy_depths;

dyngen!(
    fn ground_model(noise: f32) -> Depths {
        let cam_y = uniform(0.5, 2.0) %= "cam/y";
        let cam_roll = normal(0.0, PI / 8.0) %= "cam/roll";

        // This specifies a camera transformation (`glam` crate)
        // using the sampled parameters and some constants.
        let cam = Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, cam_roll),
            [0.0, cam_y, 1.2].into(),
        );

        // EXERCISE: use ModPPL's other primitive operation: `/=`.
        // factor out `cam` sampling logic into `cam_pose` below,
        // and uncomment the following line.
        // let cam = cam_pose() /= "cam";

        // the scene: an infinite ground plane
        let ground = Box::new(Plane {
            origin: Vec3A::ZERO,
            normal: [0.0, 1.0, 0.0].into(),
        }) as Box<dyn Solid>;

        // the renderer *is* the likelihood's mean: raytrace a depth image, then
        // observe a noisy version of it.
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![0.0; AREA()];
        raytrace_depths(cam, proj, &vec![ground], &mut pixels);

        // The observation is a noisy version of the clean image.
        // Adds a truncated gaussian perturbation to each pixel.
        noisy_depths(pixels.clone(), noise) %= "observation";

        pixels
    }
);

dyngen!(
    fn cam_pose() -> Affine3A {
        panic!("not implemented");
    }
);

// ─── inference ───────────────────────────────────────────────────────────────

fn main() {
    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam/y");
    // cam_pass.visit("cam/roll");  // uncomment, watch inference improve

    let kernel = &InferenceKernel::new(&ground_model)
        .regen_mh(&cam_pass);

    // this is a closure that runs `kernel` in a loop,
    // and displays two panels side-by-side:
    //   [observation, trace hypothesis]
    run_sandbox_loop(
        "tutorial 02: ground  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let gt = ground_model.simulate(NOISE);
            let synth_obs = gt.data.read::<Depths>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(synth_obs.clone())); // yes, Arc is... you get the point.
            let trace = ground_model.generate(NOISE, constraints).0;

            let mut printed_lines = 0usize;
            let mut trace = trace;
            move |_keys: &[minifb::Key]| {
                // slow the sweep down so each step is actually visible/
                // readable live. tune or remove for a faster demo.
                std::thread::sleep(std::time::Duration::from_millis(100));

                trace = kernel.step(trace.clone());
                print_trace_live(&trace, &mut printed_lines);

                Panels::Depth {
                    obs: synth_obs.clone(),
                    hyp: trace.retv.clone().unwrap(),
                }
            }
        },
    )
    .expect("window error");
}
