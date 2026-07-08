//! Live-window demo of a sphere + color model: sphere position and color
//! under unknown lighting.
//!
//! Run it:
//!   cargo run --release --example sandbox -- sphere
//! Keys: Space = pause, R = resample, S = snapshot, ESC = quit.

#![allow(non_upper_case_globals)]

use std::f32::consts::PI;

use glam::{Affine3A, EulerRot, Mat4, Quat, Vec3A};
use modppl::prelude::*;
use modppl_derender::{
    config::{AREA, FAR, FOVY, H, NEAR, W},
    core::*,
    image::Colors,
    inference::InferenceKernel,
    sandbox::{run_sandbox_loop, Panels},
};

// factored into its own sub-generative function, called via `/=` -- see
// examples/scenes/cone_sphere.rs for the pattern in more depth. `cam_pass`
// below regenerates both latents as one whole address, "cam".
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
    fn sphere_color_model() -> Colors {
        let x = camera_model() /= "cam";

        // background
        let brightness = uniform(0.5, 1.0) %= "ambient_brightness";
        let background_color = [brightness, brightness, brightness];

        // ground
        let ground_albedo = uniform(0.0, 1.0) %= "ground_albedo";
        let ground = (
            Box::new(Plane {
                origin: Vec3A::ZERO,
                normal: [0.0, 1.0, 0.0].into(),
            }) as Box<dyn Solid>,
            [ground_albedo, ground_albedo, ground_albedo],
        );

        // sphere
        let u = uniform(-2.0, 2.0) %= "sphere_u";
        let v = uniform(-2.0, 0.0) %= "sphere_v";
        let redness = uniform(0.0, 1.0) %= "sphere_redness";
        let sphere = (
            Box::new(Sphere {
                center: [u, 0.5, v].into(),
                radius: 0.5,
            }) as Box<dyn Solid>,
            [0.2, 1.0 - redness, redness],
        );

        // render
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![[0.0; 3]; AREA()];
        raytrace_colors(
            x,
            proj,
            &vec![ground, sphere],
            background_color,
            &mut pixels,
        );
        noisy_colors(pixels.clone(), 0.1) %= "observation";

        pixels
    }
);

pub fn run() {
    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam");

    let mut pos_pass = AddrMap::new();
    pos_pass.visit("sphere_u");
    pos_pass.visit("sphere_v");

    let mut env_pass = AddrMap::new();
    env_pass.visit("ground_albedo");
    env_pass.visit("ambient_brightness");

    let mut color_pass = AddrMap::new();
    color_pass.visit("sphere_redness");

    run_sandbox_loop(
        "derender: sphere  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let mut synth = DynTrie::new();
            synth.observe("cam/roll", Arc::new(0.0_f32));
            synth.observe("ground_albedo", Arc::new(0.5_f32));
            synth.observe("ambient_brightness", Arc::new(0.95_f32));
            let gt = sphere_color_model.generate((), synth).0;
            let observation = gt.data.read::<Colors>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone()));
            let trace = sphere_color_model.generate((), constraints).0;

            let kernel = InferenceKernel::new(&sphere_color_model)
                .regen_mh(&cam_pass)
                .regen_mh(&pos_pass)
                .regen_mh(&pos_pass)
                .regen_mh(&pos_pass)
                .regen_mh(&env_pass)
                .regen_mh(&color_pass);

            let mut trace = trace;
            move |_keys: &[minifb::Key]| {
                trace = kernel.step(trace.clone());
                Panels::Color {
                    obs: observation.clone(),
                    hyp: trace.retv.clone().unwrap(),
                }
            }
        },
    )
    .expect("window error");
}
