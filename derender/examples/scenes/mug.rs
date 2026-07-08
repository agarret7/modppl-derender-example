//! Live-window demo of a mug model (a cylinder with unknown pose, size, and
//! color).
//!
//! Run it:
//!   cargo run --release --example sandbox -- mug
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

// factored into sub-generative functions, called via `/=` -- see
// examples/scenes/cone_sphere.rs for the pattern in more depth.
dyngen!(
    fn camera_model() -> Affine3A {
        let cam_y = uniform(0.5, 2.0) %= "y";
        let cam_yaw = normal(0.0, PI / 8.0) %= "yaw";
        Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, cam_yaw, 0.0, 0.0),
            [0.0, cam_y, 1.2].into(),
        )
    }
);

dyngen!(
    fn uniform_color(lb: f32, ub: f32) -> [f32; 3] {
        let c0 = uniform(lb, ub) %= "c0";
        let c1 = uniform(lb, ub) %= "c1";
        let c2 = uniform(lb, ub) %= "c2";
        [c0, c1, c2]
    }
);

dyngen!(
    fn mug_params_model() -> (f32, f32, f32, f32, [f32; 3]) {
        let u = uniform(-1.0, 1.0) %= "u";
        let v = uniform(-1.0, 0.0) %= "v";
        let radius = uniform(0.2, 0.4) %= "radius";
        let height = uniform(0.4, 0.8) %= "height";
        let color = uniform_color(0.25, 1.0) /= "color";
        (u, v, radius, height, color)
    }
);

dyngen!(
    fn mug_model(noise: f32) -> Colors {
        let x = camera_model() /= "cam";

        // background
        let b = uniform(0.75, 1.0) %= "ambient_brightness";
        let background_c = [0.9 * b, 1.0 * b, 1.0 * b];

        // table
        let table_c = uniform_color(0.0, 1.0) /= "table";
        let table = (
            Box::new(Plane {
                origin: Vec3A::ZERO,
                normal: [0.0, 1.0, 0.0].into(),
            }) as Box<dyn Solid>,
            table_c,
        );

        // mug: a cylinder standing on the table, with unknown color and pose
        let (u, v, mug_radius, mug_height, mug_c) = mug_params_model() /= "mug";
        let mug = (
            Box::new(Cylinder {
                base: [u, 0.0, v].into(),
                radius: mug_radius,
                height: mug_height,
            }) as Box<dyn Solid>,
            mug_c,
        );

        // render
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![[0.0; 3]; AREA()];
        raytrace_colors(x, proj, &vec![table, mug], background_c, &mut pixels);
        noisy_colors(pixels.clone(), noise) %= "observation";

        pixels
    }
);

pub fn run() {

    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam");

    let mut env_pass = AddrMap::new();
    env_pass.visit("table");
    env_pass.visit("ambient_brightness");

    let mut mug_pass = AddrMap::new();
    mug_pass.visit("mug/u");
    mug_pass.visit("mug/v");
    mug_pass.visit("mug/radius");
    mug_pass.visit("mug/height");

    let mut color_pass = AddrMap::new();
    color_pass.visit("mug/color");

    let kernel = &InferenceKernel::new(&mug_model)
        .regen_mh(&cam_pass)
        .regen_mh(&env_pass)
        .regen_mh(&mug_pass)
        .mh(
            &noise_drift,
            (pass_of(&["mug/u", "mug/v", "mug/radius", "mug/height"]), 0.1),
        )
        .regen_mh(&color_pass);

    run_sandbox_loop(
        "derender: mug  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let mut synth = DynTrie::new();
            synth.observe("cam/yaw", Arc::new(0.0_f32));
            let gt = mug_model.generate(0.05_f32, synth).0;
            let observation = gt.data.read::<Colors>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone()));
            let trace = mug_model.generate(0.05_f32, constraints).0;

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
