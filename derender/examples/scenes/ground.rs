//! Live-window demo of a depth-only ground-plane model: infer the camera's
//! height and roll from a depth image of an infinite ground plane. See
//! `tutorial_02_ground.rs` if you want the from-scratch, fully-commented
//! teaching version instead.
//!
//! Run it:
//!   cargo run --release --example sandbox -- ground
//! Keys: Space = pause, R = resample, S = snapshot, ESC = quit.

#![allow(non_upper_case_globals)]

use std::f32::consts::PI;

use glam::{Affine3A, EulerRot, Mat4, Quat, Vec3A};
use modppl::prelude::*;
use modppl_derender::{
    config::{AREA, FAR, FOVY, H, NEAR, W},
    core::*,
    image::Depths,
    inference::InferenceKernel,
    sandbox::{run_sandbox_loop, Panels},
};

// factored into its own sub-generative function -- called with `/=` instead
// of `%=` since it's a whole generative function, not a single random choice.
// The moves in run() below treat "cam" as one address, regenerating both
// latents together.
dyngen!(
    fn camera_model() -> Affine3A {
        let cam_y = uniform(0.5, 2.0) %= "y";
        let cam_roll = normal(0.0, PI / 8.0) %= "roll";
        Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, cam_roll),
            [0.0, cam_y, 1.2].into(),
        )
    }
);

dyngen!(
    fn grounded_depth_model() -> Depths {
        let x = camera_model() /= "cam";

        // ground
        let ground = Box::new(Plane {
            origin: Vec3A::ZERO,
            normal: [0.0, 1.0, 0.0].into(),
        }) as Box<dyn Solid>;

        // render
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![0.0; AREA()];
        raytrace_depths(x, proj, &vec![ground], &mut pixels);
        noisy_depths(pixels.clone(), 0.1) %= "observation";

        pixels
    }
);

pub fn run() {
    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam");

    run_sandbox_loop(
        "derender: ground  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let mut synth = DynTrie::new();
            synth.observe("cam/roll", Arc::new(0.0_f32));
            synth.observe("cam/y", Arc::new(1.5_f32));
            let gt = grounded_depth_model.generate((), synth).0;
            let observation = gt.data.read::<Depths>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone()));
            let trace = grounded_depth_model.generate((), constraints).0;

            let kernel = InferenceKernel::new(&grounded_depth_model).regen_mh(&cam_pass);

            let mut trace = trace;
            move |_keys: &[minifb::Key]| {
                trace = kernel.step(trace.clone());
                Panels::Depth {
                    obs: observation.clone(),
                    hyp: trace.retv.clone().unwrap(),
                }
            }
        },
    )
    .expect("window error");
}
