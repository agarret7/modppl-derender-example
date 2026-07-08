//! Live-window demo of a ball model: a ball on a table with unknown pose and
//! color. `test_derender_ball` derenders an actual photograph
//! (`tests/ball.bmp`) against this same model, not a synthetic observation.
//!
//! Run it:
//!   cargo run --release --example sandbox -- ball
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
    fn ball_params_model() -> (f32, f32, f32, [f32; 3]) {
        let u = uniform(-1.0, 1.0) %= "u";
        let v = uniform(-1.0, 0.0) %= "v";
        let radius = uniform(0.3, 0.5) %= "radius";
        let color = uniform_color(0.25, 1.0) /= "color";
        (u, v, radius, color)
    }
);

dyngen!(
    fn ball_model() -> Colors {
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

        // ball
        let (u, v, ball_r, ball_c) = ball_params_model() /= "ball";
        let ball = (
            Box::new(Sphere {
                center: [u, ball_r, v].into(),
                radius: ball_r,
            }) as Box<dyn Solid>,
            ball_c,
        );

        // render
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![[0.0; 3]; AREA()];
        raytrace_colors(x, proj, &vec![table, ball], background_c, &mut pixels);
        noisy_colors(pixels.clone(), 0.1) %= "observation";

        pixels
    }
);

pub fn run() {
    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam");

    let mut env_pass = AddrMap::new();
    env_pass.visit("table");
    env_pass.visit("ambient_brightness");

    let mut ball_pass = AddrMap::new();
    ball_pass.visit("ball/u");
    ball_pass.visit("ball/v");
    ball_pass.visit("ball/radius");

    let mut ball_color_pass = AddrMap::new();
    ball_color_pass.visit("ball/color");

    let mut table_color_pass = AddrMap::new();
    table_color_pass.visit("table/c0");
    table_color_pass.visit("table/c1");
    table_color_pass.visit("table/c2");

    run_sandbox_loop(
        "derender: ball  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let mut synth = DynTrie::new();
            synth.observe("cam/yaw", Arc::new(0.0_f32));
            let gt = ball_model.generate((), synth).0;
            let observation = gt.data.read::<Colors>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone()));
            let trace = ball_model.generate((), constraints).0;

            let kernel = InferenceKernel::new(&ball_model)
                .regen_mh(&cam_pass)
                .regen_mh(&env_pass)
                .mh(&gaussian_drift, (table_color_pass.clone(), 0.1))
                .regen_mh(&ball_pass)
                .mh(&gaussian_drift, (ball_pass.clone(), 0.1))
                .regen_mh(&ball_color_pass);

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
