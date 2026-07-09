//! Tutorial 03: adding an object, and why block resampling isn't enough.
//!
//! The scene gains a cube resting on the ground, with unknown position and
//! size (three new latents). Two new ideas over tutorial 02:
//!
//!   1. extending the *model* is just more sampled values feeding the same
//!      renderer. the inference code doesn't know or care what the latents
//!      mean,
//!   2. block resampling (`regen_mh`) finds the right *region* fast but then
//!      keeps proposing from the whole prior forever; a *drift* proposal
//!      polishes locally. You'll enable it and watch convergence improve.
//!
//! Run it:
//!   cargo run --release --example tutorial_03_object
//!
//! Keys: Space = pause, R = fresh scene, S = snapshot, ESC = quit.

#![allow(non_upper_case_globals)]

use std::f32::consts::PI;

use glam::{Affine3A, EulerRot, Mat4, Quat, Vec3A};
use modppl::prelude::*;
use modppl_derender::{
    config::{AREA, FAR, FOVY, H, NEAR, W},
    core::*,
    image::Depths,
    inference::InferenceKernel,
    noisy_depths, print_trace_live,
    sandbox::{run_sandbox_loop, Panels},
};

const NOISE: f32 = 0.05;

// ─── the model: tutorial 02 + a cube ─────────────────────────────────────────

dyngen!(
    fn grounded_cube_model(noise: f32) -> Depths {
        let cam_y = uniform(0.5, 2.0) %= "cam/y";
        let cam_yaw = normal(-0.2, PI / 16.0) %= "cam/yaw";
        let x = Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, cam_yaw, 0.0, 0.0),
            [0.0, cam_y, 1.2].into()
        );

        let ground = Box::new(Plane {
            origin: Vec3A::ZERO,
            normal: [0.0, 1.0, 0.0].into(),
        }) as Box<dyn Solid>;

        let u = uniform(-1.0, 1.0) %= "cube/u";
        let v = uniform(-1.0, 0.0) %= "cube/v";

        // EXERCISE: give the cube a random height
        let y = 0.0;
        // let y = uniform(0.0, 1.0) %= "cube/y";

        let half_extent = uniform(0.15, 0.5) %= "cube/size";
        let cube = Box::new(Cube {
            center: [u, half_extent + y, v].into(),
            half_extent,
        }) as Box<dyn Solid>;

        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![0.0; AREA()];
        raytrace_depths(x, proj, &vec![ground, cube], &mut pixels);
        noisy_depths(pixels.clone(), noise) %= "observation";

        pixels
    }
);

// ─── a custom proposal: proposals are generative functions too ───────────────
//
// This drift proposal re-proposes each passed address from a small Gaussian
// centered on its *current* value in the trace -- a local step, where
// `regen_mh` is a global jump from the prior. Note the shape: it takes the
// current trace (weakly), reads values out of it, and makes new named
// choices. The MH machinery handles the accept/reject arithmetic.

dyngen!(
    fn drift(
        trace: Weak<DynTrace<f32, Depths>>, // Note: Weak<Trace<...>> is *required* as first proposal arg
        pass: AddrMap,
        stdev: f32,
    ) {
        // Note: this may be improved later as &DynTrace<f32, Depths>.
        // For now, it's just a one-liner to upgrade the referance.
        let trace = trace.upgrade().unwrap();

        for addr in &addrs_of(&pass) {
            // You can propose new values by sampling at addresses in `trace`.
            normal(trace.data.read::<f32>(addr), stdev) %= addr.as_str();
        }
    }
);

// ─── inference ───────────────────────────────────────────────────────────────

fn main() {
    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam/y");
    cam_pass.visit("cam/yaw");

    // EXERCISE: uncomment these
    let mut cube_pass = AddrMap::new();
    // cube_pass.visit("cube/u");
    // cube_pass.visit("cube/v");
    cube_pass.visit("cube/size");

    let kernel = &InferenceKernel::new(&grounded_cube_model)
        .regen_mh(&cam_pass)
        .regen_mh(&cube_pass)
        .mh(&drift, (cube_pass.clone(), 0.05))
        ;

    run_sandbox_loop(
        "tutorial 03: cube  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let gt = grounded_cube_model.simulate(NOISE);
            let synth_obs = gt.data.read::<Depths>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(synth_obs.clone()));
            let trace = grounded_cube_model.generate(NOISE, constraints).0;

            let mut printed_lines = 0usize;
            let mut trace = trace;
            move |_keys: &[minifb::Key]| {
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
