//! Tutorial 04: a second observation channel.
//!
//! The cube becomes a Rubik's cube: the renderer now also produces a color
//! image (each face a fixed color, no lighting), and the model observes
//! *both* channels. The headline lesson: adding an observation channel is one
//! render call plus one `%=` line -- the posterior just gets more constrained.
//!
//! Depth and color carry complementary information here:
//!   - depth identifies *where* the cube is and how big it is,
//!   - color identifies *which way it faces* (the face colors break the
//!     cube's rotational symmetry) and what the ground looks like.
//!
//! Run it:
//!   cargo run --release --example tutorial_04_color
//!
//! Window is a 2x2 grid: observed depth | observed color over hypothesis
//! depth | hypothesis color. Keys: Space, R, S, ESC as before.

#![allow(non_upper_case_globals)]

use std::f32::consts::PI;

use glam::{Affine3A, EulerRot, Mat4, Quat, Vec3A};
use modppl::prelude::*;
use modppl_derender::{
    config::{AREA, FAR, FOVY, H, NEAR, W},
    core::*,
    image::{rgb, Colors, Depths},
    inference::InferenceKernel,
    print_trace_live,
    sandbox::{run_sandbox_loop, Panels},
};

// EXERCISE (channel ablation): set COLOR_NOISE to 0.99 -- the color channel
// becomes uninformative and inference runs on depth alone: position and size
// still converge, but the ground color wanders and the cube's facing is
// whatever depth allows. Then instead set DEPTH_NOISE to 0.99: color alone
// must carry everything, and position gets mushier (many depths project to
// similar silhouettes).
const DEPTH_NOISE: f32 = 0.1;
const COLOR_NOISE: f32 = 0.1;

// ─── likelihoods: one per channel, same pattern ──────────────────────────────

use modppl_derender::{
    noisy_depths,
    noisy_colors
};

// ─── the model: tutorial 03 + color ──────────────────────────────────────────

dyngen!(
    fn rubiks_scene_model(noise: (f32, f32)) -> (Depths, Colors) {
        let (depth_noise, color_noise) = noise;

        let cam_y = uniform(0.5, 2.0) %= "cam_y";
        // slight downward bias, tight spread: the camera sits 0.5-2.0 units up,
        // so a level-or-upward pitch shows mostly sky and no scene at all
        let cam_yaw = normal(-0.2, PI / 16.0) %= "cam_yaw";
        let x = Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, cam_yaw, 0.0, 0.0),
            [0.0, cam_y, 1.2].into(),
        );

        // the ground now has an unknown color (three more latents: c0=B, c1=G,
        // c2=R. see `image::rgb` if you want to hardcode a color literal
        // instead, since a normal [f32; 3] array is in BGR order, for .bmp compat).
        // The cube needs none: `Cube::color_at` assigns fixed Rubik's face
        // colors, so the scene-assigned color below is ignored for it.
        let ground_c0 = uniform(0.0, 1.0) %= "ground/c0";
        let ground_c1 = uniform(0.0, 1.0) %= "ground/c1";
        let ground_c2 = uniform(0.0, 1.0) %= "ground/c2";
        let ground_c = rgb(ground_c0, ground_c1, ground_c2);
        let ground = (
            Box::new(Plane {
                origin: Vec3A::ZERO,
                normal: [0.0, 1.0, 0.0].into(),
            }) as Box<dyn Solid>,
            ground_c
        );

        let u = uniform(-1.0, 1.0) %= "cube_u";
        let v = uniform(-1.0, 0.0) %= "cube_v";
        // floor at 0.15 so the cube is never just a few pixels
        let half_extent = uniform(0.15, 0.5) %= "cube_size";
        let cube = (
            Box::new(Cube {
                center: [u, half_extent, v].into(),
                half_extent,
            }) as Box<dyn Solid>,
            rgb(0.0, 0.0, 0.0), // ignored: Cube::color_at overrides per-face
        );

        // one shared ray-cast pass fills both images (flat = no lighting, fully
        // deterministic. MCMC needs the likelihood to be a *function* of the
        // latents, and a Monte Carlo renderer adds noise to the log-probability).
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut depths = vec![0.0; AREA()];
        let mut colors = vec![[0.0; 3]; AREA()];
        raytrace_depths_and_flat_colors(
            x,
            proj,
            &vec![ground, cube],
            rgb(0.7, 0.7, 0.7),
            &mut depths,
            &mut colors,
        );

        // two observation channels: two `%=` lines. That's the whole change.
        noisy_depths(depths.clone(), depth_noise) %= "observation/depths";
        noisy_colors(colors.clone(), color_noise) %= "observation/colors";
        (depths, colors)
    }
);

dyngen!(
    fn drift(trace: Weak<DynTrace<(f32, f32), (Depths, Colors)>>, pass: AddrMap, stdev: f32) {
        let trace = trace.upgrade().unwrap();
        for addr in &addrs_of(&pass) {
            normal(trace.data.read::<f32>(addr), stdev) %= addr.as_str();
        }
    }
);

// ─── inference ───────────────────────────────────────────────────────────────

fn main() {
    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam_y");
    cam_pass.visit("cam_yaw");

    let mut cube_pass = AddrMap::new();
    cube_pass.visit("cube_u");
    cube_pass.visit("cube_v");
    cube_pass.visit("cube_size");

    let mut ground_pass = AddrMap::new();
    ground_pass.visit("ground");

    let kernel = &InferenceKernel::new(&rubiks_scene_model)
        .regen_mh(&cam_pass)
        .regen_mh(&cube_pass)
        .mh(&drift, (cube_pass.clone(), 0.05))
        .regen_mh(&ground_pass);

    // you're nearly done with this tutorial.
    // afterward, try running
    // `cargo run --release --example sandbox rubiks`
    run_sandbox_loop(
        "tutorial 04: rubiks  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let gt = rubiks_scene_model
                .generate((DEPTH_NOISE, COLOR_NOISE), DynTrie::new())
                .0;
            let obs_depth = gt.data.read::<Depths>("observation/depths").clone();
            let obs_color = gt.data.read::<Colors>("observation/colors").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation/depths", Arc::new(obs_depth.clone()));
            constraints.observe("observation/colors", Arc::new(obs_color.clone()));
            let trace = rubiks_scene_model
                .generate((DEPTH_NOISE, COLOR_NOISE), constraints)
                .0;

            let mut printed_lines = 0usize;
            let mut trace = trace;
            move |_keys: &[minifb::Key]| {
                trace = kernel.step(trace.clone());
                print_trace_live(&trace, &mut printed_lines);

                let (hyp_depth, hyp_color) = trace.retv.clone().unwrap();
                Panels::Rgbd {
                    obs: (obs_depth.clone(), obs_color.clone()),
                    hyp: (hyp_depth, hyp_color),
                }
            }
        },
    )
    .expect("window error");
}
