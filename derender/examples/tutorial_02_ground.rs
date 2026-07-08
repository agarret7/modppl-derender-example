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

dyngen!(
    fn ground_model(noise: f32) -> Depths {
        // `dyngen!` lines with "%=" are *sample* statements,
        // they represent the string-addressable random choices,
        // and are sampled from a *prior* distribution.
        let cam_y = uniform(0.5, 2.0) %= "cam/y";
        let cam_roll = normal(0.0, PI / 8.0) %= "cam/roll";

        // This specifies a camera transformation (`glam` crate)
        // using the sampled parameters and some constants.
        let x = Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, cam_roll),
            [0.0, cam_y, 1.2].into(),
        );

        // the scene: one infinite ground plane (color is unused in depth renders)
        let ground = Box::new(Plane {
            origin: Vec3A::ZERO,
            normal: [0.0, 1.0, 0.0].into(),
        }) as Box<dyn Solid>;

        // the renderer *is* the likelihood's mean: raytrace a depth image, then
        // observe a noisy version of it.
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![0.0; AREA()];
        raytrace_depths(x, proj, &vec![ground], &mut pixels);

        // The observation is a noisy version of the clean image.
        noisy_depths(pixels.clone(), noise) %= "observation";

        pixels
    }
);

// ─── the likelihood ───────────────────────────────────────────────────────
//
// A likelihood in ModPPL is simple: any custom type implementing
// `Distribution<Value, Params>` is a sampleable base distribution (%=)
// that implements two functions: Distribution::logpdf and Distribution::random
// Read more about them: github.com/agarret7/modppl/tree/main/modppl/src/modeling/dists

// This one scores an observed depth image against a rendered one,
// pixel by pixel, under truncated-Gaussian pixel noise (truncated to [0,1],
// the renderer's normalized depth range).
use modppl_derender::noisy_depths;

fn main() {
    // ─── 1. simulate ────────────────────────────────────────────────────
    let trace = ground_model.simulate(NOISE);

    // print_dyntrace already includes the trace's data (every named random
    // choice, i.e. the trie) in its output -- there's no separate "print the
    // trie alone" function needed.
    println!(
        "{}",
        modppl::dyntrie_to_string_with(
            &trace.data,
            &[] // &[dyn_debug_formatter::<Depths>]
        )
    );

    // ─── 2. print its values ────────────────────────────────────────────
    // any named random choice can be read straight out of the trace.

    // the SAFE typed read pattern (see tutorial_01 for when/why this panics).
    let mut cam_y = trace.data.read::<f32>("cam/y");
    let cam_roll = trace.data.read::<f32>("cam/roll");

    println!("sampled cam/y:    {:.3}", cam_y);
    println!("sampled cam/roll: {:.3}", cam_roll);

    // this is the UNSAFE autocast access pattern.
    unsafe {
        cam_y = trace.data.auto("cam/y");
    }

    println!("sampled cam/y:    {:.3}", cam_y);
    println!("sampled cam/roll: {:.3}", cam_roll);

    // ─── 3. render the observed image ──────────────────────────────────
    // the model's return value IS the rendered depth image; "observation" is
    // that same image after going through the noise model. tutorial_01 had
    // five numbers to print; here the "measurement" is a whole image, so we
    // print a summary instead of all ~4k pixels at RES=64
    let observation = trace.data.read::<Depths>("observation").clone();
    let (lo, hi) = observation
        .iter()
        .fold((f32::MAX, f32::MIN), |(lo, hi), &v| (lo.min(v), hi.max(v)));
    println!(
        "rendered a {} pixel depth image, range [{lo:.3}, {hi:.3}]",
        observation.len()
    );

    // ─── 4. condition ───────────────────────────────────────────────────
    // a *new* trace, constrained only on the observed image.
    let mut constraints = DynTrie::new();
    constraints.observe("observation", Arc::new(observation.clone())); // yes, Arc is required.
    let (_trace, _weight) = ground_model.generate(NOISE, constraints);

    // ─── 5. regenerate choices ──────────────────────────────────────────
    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam/y");
    // cam_pass.visit("cam/roll");  // uncomment, watch inference improve

    let kernel = &InferenceKernel::new(&ground_model).regen_mh(&cam_pass);

    // this is a closure that runs `kernel` in a loop,
    // and displays two panels side-by-side:
    //   [observation, trace hypothesis]
    run_sandbox_loop(
        "tutorial 02: ground  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let trace = ground_model.simulate(NOISE);
            let observation = trace.data.read::<Depths>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone())); // yes, Arc is... you get the point.
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
                    obs: observation.clone(),
                    hyp: trace.retv.clone().unwrap(),
                }
            }
        },
    )
    .expect("window error");
}
