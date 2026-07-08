#![allow(non_upper_case_globals)]

use glam::{Affine3A, EulerRot, Mat4, Quat, Vec3A};
use modppl::prelude::*;
use modppl_derender::{
    config::{apply_cube_pipeline_defaults, AREA, FAR, FOVY, H, NEAR, W},
    core::*,
    image::*,
    inference::InferenceKernel,
    serialization::*,
};
use std::f32::consts::PI;
use std::fs::create_dir_all;

// the example-only scene models (grounded_depth_model, sphere_color_model,
// ball_model, mug_model, cone_sphere_model) live in their matching
// examples/scenes/*.rs files, not in the library -- each gets its own inline
// copy here too, same tradeoff as the examples themselves. cube_rgbd_model is
// the one production model with real cross-crate consumers, so it stays
// importable from core.

dyngen!(
    fn grounded_depth_model() -> Depths {
        let cam_y = uniform(0.5, 2.0) %= "cam_y";
        let cam_roll = normal(0.0, PI / 8.0) %= "cam_roll";
        let x = Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, cam_roll),
            [0.0, cam_y, 1.2].into(),
        );

        let ground = Box::new(Plane {
                origin: Vec3A::ZERO,
                normal: [0.0, 1.0, 0.0].into(),
            }) as Box<dyn Solid>;

        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![0.0; AREA()];
        raytrace_depths(x, proj, &vec![ground], &mut pixels);
        noisy_depths(pixels.clone(), 0.1) %= "observation";

        pixels
    }
);

dyngen!(
    fn sphere_color_model() -> Colors {
        let cam_y = uniform(0.5, 2.0) %= "cam_y";
        let cam_roll = normal(0.0, PI / 8.0) %= "cam_roll";
        let x = Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, cam_roll),
            [0.0, cam_y, 1.2].into(),
        );

        let brightness = uniform(0.5, 1.0) %= "ambient_brightness";
        let background_color = [brightness, brightness, brightness];

        let ground_albedo = uniform(0.0, 1.0) %= "ground_albedo";
        let ground = (
            Box::new(Plane {
                origin: Vec3A::ZERO,
                normal: [0.0, 1.0, 0.0].into(),
            }) as Box<dyn Solid>,
            [ground_albedo, ground_albedo, ground_albedo],
        );

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

dyngen!(
    fn ball_model() -> Colors {
        let cam_y = uniform(0.5, 2.0) %= "cam_y";
        let cam_yaw = normal(0.0, PI / 8.0) %= "cam_yaw";
        let x = Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, cam_yaw, 0.0, 0.0),
            [0.0, cam_y, 1.2].into(),
        );

        let b = uniform(0.75, 1.0) %= "ambient_brightness";
        let background_c = [0.9 * b, 1.0 * b, 1.0 * b];

        let mut table_c = [0.0; 3];
        table_c[0] = uniform(0.0, 1.0) %= "table_c0";
        table_c[1] = uniform(0.0, 1.0) %= "table_c1";
        table_c[2] = uniform(0.0, 1.0) %= "table_c2";
        let table = (
            Box::new(Plane {
                origin: Vec3A::ZERO,
                normal: [0.0, 1.0, 0.0].into(),
            }) as Box<dyn Solid>,
            table_c,
        );

        let u = uniform(-1.0, 1.0) %= "ball_u";
        let v = uniform(-1.0, 0.0) %= "ball_v";
        let mut ball_c = [0.0; 3];
        ball_c[0] = uniform(0.25, 1.0) %= "ball_c0";
        ball_c[1] = uniform(0.25, 1.0) %= "ball_c1";
        ball_c[2] = uniform(0.25, 1.0) %= "ball_c2";
        let ball_r = uniform(0.3, 0.5) %= "ball_radius";
        let ball = (
            Box::new(Sphere {
                center: [u, ball_r, v].into(),
                radius: ball_r,
            }) as Box<dyn Solid>,
            ball_c,
        );

        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![[0.0; 3]; AREA()];
        raytrace_colors(x, proj, &vec![table, ball], background_c, &mut pixels);
        noisy_colors(pixels.clone(), 0.1) %= "observation";

        pixels
    }
);

dyngen!(
    fn mug_model(noise: f32) -> Colors {
        let cam_y = uniform(0.5, 2.0) %= "cam_y";
        let cam_yaw = normal(0.0, PI / 8.0) %= "cam_yaw";
        let x = Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, cam_yaw, 0.0, 0.0),
            [0.0, cam_y, 1.2].into(),
        );

        let b = uniform(0.75, 1.0) %= "ambient_brightness";
        let background_c = [0.9 * b, 1.0 * b, 1.0 * b];

        let mut table_c = [0.0; 3];
        table_c[0] = uniform(0.0, 1.0) %= "table_c0";
        table_c[1] = uniform(0.0, 1.0) %= "table_c1";
        table_c[2] = uniform(0.0, 1.0) %= "table_c2";
        let table = (
            Box::new(Plane {
                origin: Vec3A::ZERO,
                normal: [0.0, 1.0, 0.0].into(),
            }) as Box<dyn Solid>,
            table_c,
        );

        let u = uniform(-1.0, 1.0) %= "mug_u";
        let v = uniform(-1.0, 0.0) %= "mug_v";
        let mug_radius = uniform(0.2, 0.4) %= "mug_radius";
        let mug_height = uniform(0.4, 0.8) %= "mug_height";
        let mut mug_c = [0.0; 3];
        mug_c[0] = uniform(0.25, 1.0) %= "mug_c0";
        mug_c[1] = uniform(0.25, 1.0) %= "mug_c1";
        mug_c[2] = uniform(0.25, 1.0) %= "mug_c2";
        let mug = (
            Box::new(Cylinder {
                base: [u, 0.0, v].into(),
                radius: mug_radius,
                height: mug_height,
            }) as Box<dyn Solid>,
            mug_c,
        );

        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![[0.0; 3]; AREA()];
        raytrace_colors(x, proj, &vec![table, mug], background_c, &mut pixels);
        noisy_colors(pixels.clone(), noise) %= "observation";

        pixels
    }
);

dyngen!(
    fn camera_model() -> Affine3A {
        let cam_y = uniform(0.1, 0.75) %= "y";
        let cam_yaw = normal(0.0, PI / 16.0) %= "yaw";
        let cam_roll = normal(0.0, PI / 16.0) %= "roll";
        Affine3A::from_rotation_translation(
            Quat::from_euler(EulerRot::XYZ, cam_yaw, 0.0, cam_roll),
            [0.0, cam_y, 1.2].into(),
        )
    }
);

dyngen!(
    fn position_model() -> (f32, f32) {
        let u = uniform(-1.0, 1.0) %= "u";
        let v = uniform(-0.7, 0.8) %= "v";
        (u, v)
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
    fn cone_params_model() -> (f32, f32, f32, f32, [f32; 3]) {
        let (cone_u, cone_v) = position_model() /= "pos";
        let cone_height = uniform(0.3, 0.8) %= "height";
        let cone_radius = uniform(0.15, 0.4) %= "radius";
        let cone_c = uniform_color(0.25, 1.0) /= "color";
        (cone_u, cone_v, cone_height, cone_radius, cone_c)
    }
);

dyngen!(
    fn sphere_params_model() -> (f32, f32, f32, [f32; 3]) {
        let (sphere_u, sphere_v) = position_model() /= "pos";
        let sphere_radius = uniform(0.15, 0.35) %= "radius";
        let sphere_c = uniform_color(0.25, 1.0) /= "color";
        (sphere_u, sphere_v, sphere_radius, sphere_c)
    }
);

dyngen!(
    fn cone_sphere_model(noise: f32) -> Colors {
        let x = camera_model() /= "cam";

        let b = uniform(0.75, 1.0) %= "ambient_brightness";
        let background_c = [0.9 * b, 1.0 * b, 1.0 * b];

        let (cone_u, cone_v, cone_height, cone_radius, cone_c) = cone_params_model() /= "cone";
        let cone = (
            Box::new(Cone {
                base: [cone_u, 0.0, cone_v].into(),
                base_radius: cone_radius,
                height: cone_height,
            }) as Box<dyn Solid>,
            cone_c,
        );

        let (sphere_u, sphere_v, sphere_radius, sphere_c) = sphere_params_model() /= "sphere";
        let sphere = (
            Box::new(Sphere {
                center: [sphere_u, sphere_radius, sphere_v].into(),
                radius: sphere_radius,
            }) as Box<dyn Solid>,
            sphere_c,
        );

        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![[0.0; 3]; AREA()];
        raytrace_colors(x, proj, &vec![cone, sphere], background_c, &mut pixels);
        noisy_colors(pixels.clone(), noise) %= "observation";

        pixels
    }
);

/// quick visual sanity check (no inference) that the Rubik's-cube face coloring
/// looks right: a single still render of the cube sitting on the table.
#[test]
fn test_render_rubiks_cube() {
    create_dir_all("out").expect("error creating 'out' dir");

    let x = Affine3A::from_rotation_translation(
        glam::Quat::from_euler(glam::EulerRot::XYZ, -0.4, 0.6, 0.0),
        [0.0, 1.2, 1.2].into(),
    );
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());

    let table = (
        Box::new(Plane {
            origin: Vec3A::ZERO,
            normal: [0.0, 1.0, 0.0].into(),
        }) as Box<dyn Solid>,
        [0.6, 0.6, 0.6],
    );
    let cube = (
        Box::new(Cube {
            center: [0.0, 0.4, -0.5].into(),
            half_extent: 0.4,
        }) as Box<dyn Solid>,
        [0.0, 0.0, 0.0], // ignored: Cube::color_at overrides per-face
    );

    let mut pixels = vec![[0.0; 3]; AREA()];
    raytrace_colors(x, proj, &vec![table, cube], [0.9, 0.9, 0.9], &mut pixels);
    save_colors("./out/rubiks_cube.bmp", &pixels);
}

/// quick visual sanity check (no inference, no noise) that the new `Cone`
/// primitive actually looks like a cone (tapered silhouette, not a second
/// sphere) sitting next to a `Sphere` on the table.
#[test]
fn test_render_cone_sphere() {
    create_dir_all("out").expect("error creating 'out' dir");

    let x = Affine3A::from_rotation_translation(
        glam::Quat::from_euler(glam::EulerRot::XYZ, -0.3, 0.0, 0.0),
        [0.0, 0.9, 1.2].into(),
    );
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());

    let table = (
        Box::new(Plane {
            origin: Vec3A::ZERO,
            normal: [0.0, 1.0, 0.0].into(),
        }) as Box<dyn Solid>,
        [0.6, 0.6, 0.6],
    );
    let cone = (
        Box::new(Cone {
            base: [-0.35, 0.0, -0.8].into(),
            base_radius: 0.3,
            height: 0.6,
        }) as Box<dyn Solid>,
        [0.2, 0.5, 0.9],
    );
    let sphere = (
        Box::new(Sphere {
            center: [0.35, 0.25, -0.8].into(),
            radius: 0.25,
        }) as Box<dyn Solid>,
        [0.9, 0.3, 0.3],
    );

    let mut pixels = vec![[0.0; 3]; AREA()];
    raytrace_colors(
        x,
        proj,
        &vec![table, cone, sphere],
        [0.9, 0.9, 0.9],
        &mut pixels,
    );
    save_colors("./out/cone_sphere_still.bmp", &pixels);
}

#[test]
fn test_derender_ground_depth() {
    create_dir_all("out").expect("error creating 'out' dir");

    // simulate constraints
    let mut synth_constraints = DynTrie::new();
    synth_constraints.observe("cam_roll", Arc::new(0.0_f32));
    synth_constraints.observe("cam_y", Arc::new(1.5_f32));
    let trace = grounded_depth_model.generate((), synth_constraints).0;

    // generate trace
    let mut constraints = DynTrie::new();
    let observation = trace.data.read::<Depths>("observation").clone();
    constraints.observe("observation", Arc::new(observation.clone()));
    let trace = grounded_depth_model.generate((), constraints).0;

    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam_roll");
    cam_pass.visit("cam_y");

    const NUM_ITERS: usize = 200;
    let renders: Vec<Depths> = InferenceKernel::new(&grounded_depth_model)
        .regen_mh(&cam_pass)
        .iter(trace)
        .take(NUM_ITERS)
        .map(|t| t.retv.clone().unwrap())
        .collect();

    let observations = vec![observation; NUM_ITERS];
    save_depths2_video("./out/ground.mp4", &observations, &renders, 20);
}

#[test]
fn test_derender_sphere_color() {
    create_dir_all("out").expect("error creating 'out' dir");

    for i in 1..=3 {
        // simulate constraints
        let mut synth_constraints = DynTrie::new();
        synth_constraints.observe("cam_roll", Arc::new(0.0_f32));
        synth_constraints.observe("ground_albedo", Arc::new(0.5_f32));
        synth_constraints.observe("ambient_brightness", Arc::new(0.95_f32));
        let trace = sphere_color_model.generate((), synth_constraints).0;

        // generate trace
        let mut constraints = DynTrie::new();
        let observation = trace.data.read::<Colors>("observation").clone();
        constraints.observe("observation", Arc::new(observation.clone()));
        let trace = sphere_color_model.generate((), constraints).0;

        let mut cam_pass = AddrMap::new();
        cam_pass.visit("cam_y");
        cam_pass.visit("cam_roll");

        let mut pos_pass = AddrMap::new();
        pos_pass.visit("sphere_u");
        pos_pass.visit("sphere_v");

        let mut env_pass = AddrMap::new();
        env_pass.visit("ground_albedo");
        env_pass.visit("ambient_brightness");

        let mut sphere_color_pass = AddrMap::new();
        sphere_color_pass.visit("sphere_redness");

        const NUM_ITERS: usize = 200;
        let renders: Vec<Colors> = InferenceKernel::new(&sphere_color_model)
            .regen_mh(&cam_pass)
            .regen_mh(&pos_pass)
            .regen_mh(&pos_pass)
            .regen_mh(&pos_pass)
            .regen_mh(&env_pass)
            .regen_mh(&sphere_color_pass)
            .iter(trace)
            .take(NUM_ITERS)
            .map(|t| t.retv.clone().unwrap())
            .collect();

        let observations = vec![observation; NUM_ITERS];
        save_colors2_video(&format!("./out/sphere{i}.mp4"), &observations, &renders, 20);
    }
}

// fixed observation noise: the mug posterior (single object, no occlusion) is
// effectively unimodal, so annealing only adds warm-up cost here with no benefit.
// Annealing is worth revisiting for multi-object scenes, where occlusion and
// explaining-away create the multimodal posteriors it's actually meant to escape.
const MUG_NOISE: f32 = 0.05;

#[test]
fn test_derender_mug() {
    create_dir_all("out").expect("error creating 'out' dir");

    // simulate constraints (fix the camera yaw so the mug stays in frame)
    let mut synth_constraints = DynTrie::new();
    synth_constraints.observe("cam_yaw", Arc::new(0.0_f32));
    let trace = mug_model.generate(MUG_NOISE, synth_constraints).0;

    // condition on the rendered observation
    let mut constraints = DynTrie::new();
    let observation = trace.data.read::<Colors>("observation").clone();
    constraints.observe("observation", Arc::new(observation.clone()));
    let trace = mug_model.generate(MUG_NOISE, constraints).0;

    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam_y");
    cam_pass.visit("cam_yaw");

    let mut env_pass = AddrMap::new();
    env_pass.visit("table_c0");
    env_pass.visit("table_c1");
    env_pass.visit("table_c2");
    env_pass.visit("ambient_brightness");

    let mut mug_pass = AddrMap::new();
    mug_pass.visit("mug_u");
    mug_pass.visit("mug_v");
    mug_pass.visit("mug_radius");
    mug_pass.visit("mug_height");

    let mut mug_color_pass = AddrMap::new();
    mug_color_pass.visit("mug_c0");
    mug_color_pass.visit("mug_c1");
    mug_color_pass.visit("mug_c2");

    const NUM_ITERS: usize = 250;
    let start = std::time::Instant::now();
    let renders: Vec<Colors> = InferenceKernel::new(&mug_model)
        .regen_mh(&cam_pass)
        .regen_mh(&env_pass)
        .regen_mh(&mug_pass)
        .mh(
            &noise_drift,
            (pass_of(&["mug_u", "mug_v", "mug_radius", "mug_height"]), 0.1),
        )
        .regen_mh(&mug_color_pass)
        .iter(trace)
        .take(NUM_ITERS)
        .enumerate()
        .inspect(|(i, _)| {
            println!(
                "iter {}/{NUM_ITERS} ({:.2?} elapsed, {:.3?}/iter)",
                i + 1,
                start.elapsed(),
                start.elapsed() / (*i as u32 + 1)
            )
        })
        .map(|(_, t)| t.retv.clone().unwrap())
        .collect();

    let observations = vec![observation; NUM_ITERS];
    save_colors2_video("./out/mug.mp4", &observations, &renders, 20);
}

// fixed noise: with two objects there's now real potential for occlusion-driven
// multimodality (see cone_sphere_model's doc comment), unlike the single-object
// models above -- worth revisiting with annealing if/when that shows up.
const CONE_SPHERE_NOISE: f32 = 0.05;
// inference runs hotter than the synth noise above (see examples/scenes/cone_sphere.rs,
// which uses the same 5x ratio) -- a looser likelihood keeps MH from getting stuck.
const CONE_SPHERE_INFERENCE_NOISE: f32 = 0.25;

/// see `examples/scenes/cone_sphere.rs` -- the same jump/coarse-to-fine
/// kernel, kept in sync with the live demo.
const CONE_SPHERE_JUMP_PROB: f32 = 0.15;
const CONE_SPHERE_DRIFT_SCALES: [f32; 3] = [0.2, 0.08, 0.02];

#[test]
fn test_derender_cone_sphere() {
    create_dir_all("out").expect("error creating 'out' dir");

    // simulate constraints: the same hand-tuned scene as examples/cone_sphere.rs
    // (see that file's comments for the reasoning behind each value)
    let mut synth_constraints = DynTrie::new();
    synth_constraints.observe("cam/yaw", Arc::new(0.0_f32));
    synth_constraints.observe("cam/roll", Arc::new(0.0_f32));
    synth_constraints.observe("cam/y", Arc::new(0.2_f32));
    synth_constraints.observe("sphere/radius", Arc::new(0.15_f32));
    synth_constraints.observe("cone/radius", Arc::new(0.225_f32));
    synth_constraints.observe("cone/height", Arc::new(0.75_f32));
    synth_constraints.observe("sphere/pos/u", Arc::new(0.15_f32));
    synth_constraints.observe("sphere/pos/v", Arc::new(0.6_f32));
    synth_constraints.observe("cone/pos/u", Arc::new(-0.05_f32));
    synth_constraints.observe("cone/pos/v", Arc::new(0.3_f32));
    synth_constraints.observe("cone/color/c0", Arc::new(0.5_f32));
    synth_constraints.observe("cone/color/c1", Arc::new(0.25_f32));
    synth_constraints.observe("cone/color/c2", Arc::new(0.5_f32));
    synth_constraints.observe("sphere/color/c0", Arc::new(0.25_f32));
    synth_constraints.observe("sphere/color/c1", Arc::new(0.25_f32));
    synth_constraints.observe("sphere/color/c2", Arc::new(1.0_f32));
    let trace = cone_sphere_model
        .generate(CONE_SPHERE_NOISE, synth_constraints)
        .0;

    // condition on the rendered observation
    let mut constraints = DynTrie::new();
    let observation = trace.data.read::<Colors>("observation").clone();
    constraints.observe("observation", Arc::new(observation.clone()));
    let trace = cone_sphere_model.generate(CONE_SPHERE_INFERENCE_NOISE, constraints).0;

    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam");

    let mut env_pass = AddrMap::new();
    env_pass.visit("ambient_brightness");

    let mut cone_pos_pass = AddrMap::new();
    cone_pos_pass.visit("cone/pos");

    let mut cone_color_pass = AddrMap::new();
    cone_color_pass.visit("cone/color");

    let mut sphere_pos_pass = AddrMap::new();
    sphere_pos_pass.visit("sphere/pos");

    let mut sphere_color_pass = AddrMap::new();
    sphere_color_pass.visit("sphere/color");

    const NUM_ITERS: usize = 400;
    let start = std::time::Instant::now();
    let kernel = InferenceKernel::new(&cone_sphere_model)
        .then(|t| {
            let mut rng = ThreadRng::default();
            if u01(&mut rng) < CONE_SPHERE_JUMP_PROB {
                regen_mh(&cone_sphere_model, t, &cam_pass)
            } else {
                mh(
                    &cone_sphere_model,
                    t,
                    &noise_drift,
                    (pass_of(&["cam/y", "cam/yaw"]), 0.03),
                )
            }
        })
        .regen_mh(&env_pass)
        .then(|t| {
            let mut rng = ThreadRng::default();
            if u01(&mut rng) < CONE_SPHERE_JUMP_PROB {
                regen_mh(&cone_sphere_model, t, &cone_pos_pass)
            } else {
                (t, false)
            }
        })
        .then(|mut t| {
            for stdev in CONE_SPHERE_DRIFT_SCALES {
                t = mh(
                    &cone_sphere_model,
                    t,
                    &noise_drift,
                    (
                        pass_of(&["cone/pos/u", "cone/pos/v", "cone/height", "cone/radius"]),
                        stdev,
                    ),
                )
                .0;
            }
            (t, false)
        })
        .then(|t| {
            let mut rng = ThreadRng::default();
            if u01(&mut rng) < CONE_SPHERE_JUMP_PROB {
                regen_mh(&cone_sphere_model, t, &cone_color_pass)
            } else {
                mh(
                    &cone_sphere_model,
                    t,
                    &noise_drift,
                    (pass_of(&["cone/color/c0", "cone/color/c1", "cone/color/c2"]), 0.05),
                )
            }
        })
        .then(|t| {
            let mut rng = ThreadRng::default();
            if u01(&mut rng) < CONE_SPHERE_JUMP_PROB {
                regen_mh(&cone_sphere_model, t, &sphere_pos_pass)
            } else {
                (t, false)
            }
        })
        .then(|mut t| {
            for stdev in CONE_SPHERE_DRIFT_SCALES {
                t = mh(
                    &cone_sphere_model,
                    t,
                    &noise_drift,
                    (pass_of(&["sphere/pos/u", "sphere/pos/v", "sphere/radius"]), stdev),
                )
                .0;
            }
            (t, false)
        })
        .then(|t| {
            let mut rng = ThreadRng::default();
            if u01(&mut rng) < CONE_SPHERE_JUMP_PROB {
                regen_mh(&cone_sphere_model, t, &sphere_color_pass)
            } else {
                mh(
                    &cone_sphere_model,
                    t,
                    &noise_drift,
                    (pass_of(&["sphere/color/c0", "sphere/color/c1", "sphere/color/c2"]), 0.05),
                )
            }
        });

    let (renders, trace_texts): (Vec<Colors>, Vec<String>) = kernel
        .iter(trace)
        .take(NUM_ITERS)
        .enumerate()
        .inspect(|(i, _)| {
            println!(
                "iter {}/{NUM_ITERS} ({:.2?} elapsed, {:.3?}/iter)",
                i + 1,
                start.elapsed(),
                start.elapsed() / (*i as u32 + 1)
            )
        })
        .map(|(_, t)| {
            // pulled from the SAME `t` in the SAME iteration as the render
            // below -- trace_texts[i] and renders[i] describe the identical
            // trace state by construction, so a text video and the render
            // video built from these two vecs at the same fps/frame count
            // stay frame-for-frame synced with no alignment step needed.
            //
            // hand-formatted, not modppl::dyntrace_to_string_with_options:
            // DynTrie's backing store is a plain HashMap, so its print order
            // is arbitrary hash-bucket order (not alphabetical, not insertion
            // order) and can vary between runs -- fine for ad hoc debugging,
            // not for a transcript meant to read narratively (cam, then each
            // object, then the render).
            // `t.data.read::<V>(addr)` on a compound (`/=`) address returns
            // the sub-trace's *retv* directly, not its internal choices --
            // trace_at's handler does `sub.replace_inner(Arc::new(retv))` on
            // every compound node, so the retv sits right alongside its
            // children in the trie. Reading "cone/pos" as (f32,f32) below
            // pulls the whole tuple in one call instead of two leaf reads.
            let f = |addr: &str| t.data.read::<f32>(addr);
            let (cone_u, cone_v) = t.data.read::<(f32, f32)>("cone/pos");
            let cone_c = t.data.read::<[f32; 3]>("cone/color");
            let (sphere_u, sphere_v) = t.data.read::<(f32, f32)>("sphere/pos");
            let sphere_c = t.data.read::<[f32; 3]>("sphere/color");
            // hand-rolled, not modppl's dyntrace printer: we want compound
            // (`/=`) nodes to show their retv ("xy"/"rgb") instead of their
            // internal `%=` choices, which the library printer has no option
            // for. Two blank lines bracket "observation" so a video-compositing
            // step can later drop the actual observed frame into that gap.
            let text = format!(
                "data: {{\n\
                 \x20 \"cam\" => {{\n\
                 \x20   \"y\" => {:.3},\n\
                 \x20   \"yaw\" => {:.3},\n\
                 \x20   \"roll\" => {:.3}\n\
                 \x20 }},\n\
                 \x20 \"ambient_brightness\" => {:.3},\n\
                 \x20 \"cone\" => {{\n\
                 \x20   \"height\" => {:.3},\n\
                 \x20   \"radius\" => {:.3},\n\
                 \x20   \"rgb\" => ({:.2}, {:.2}, {:.2}),\n\
                 \x20   \"xy\" => ({cone_u:.3}, {cone_v:.3})\n\
                 \x20 }},\n\
                 \x20 \"sphere\" => {{\n\
                 \x20   \"radius\" => {:.3},\n\
                 \x20   \"rgb\" => ({:.2}, {:.2}, {:.2}),\n\
                 \x20   \"xy\" => ({sphere_u:.3}, {sphere_v:.3})\n\
                 \x20 }}\n\
                 \n\
                 \x20 \"observation\" => \n\
                 \n\
                 }}\n\
                 \n\
                 logjp: {:.1}",
                f("cam/y"), f("cam/yaw"), f("cam/roll"),
                f("ambient_brightness"),
                f("cone/height"), f("cone/radius"), cone_c[0], cone_c[1], cone_c[2],
                f("sphere/radius"), sphere_c[0], sphere_c[1], sphere_c[2],
                t.logjp,
            );
            (t.retv.clone().unwrap(), text)
        })
        .unzip();

    let observations = vec![observation; NUM_ITERS];
    save_colors2_video("./out/cone_sphere.mp4", &observations, &renders, 20);

    let texts_path = "./out/cone_sphere_trace.txt";
    std::fs::write(
        texts_path,
        // one record per iteration, delimited so a video-gen script can split
        // reliably even though each trace-text snapshot is itself multi-line
        trace_texts.join("\n\x1E\n"),
    )
    .expect("error writing trace text transcript");
    println!("wrote {} trace-text frames to {texts_path}", trace_texts.len());
}

// fixed observation noise, no annealing: same reasoning as the mug model
// (single object, unimodal posterior).
const RUBIKS_NOISE: (f32, f32) = (0.05, 0.05);

/// the one cube model, end-to-end: same `cube_rgbd_model` (real-world scale,
/// depth+color observations) as the live RealSense demos and the CNN training
/// data -- here with a synthetic observation and the path-traced renderer for
/// the gallery video. Set `path_trace=false` to test the flat variant.
#[test]
fn test_derender_rubiks() {
    apply_cube_pipeline_defaults();
    create_dir_all("out").expect("error creating 'out' dir");
    let (dn, cn) = RUBIKS_NOISE;
    let path_trace = true;

    // orbital camera always points at the cube, so no constraints needed to keep
    // it in frame -- sample everything from the prior.
    let trace = cube_rgbd_model
        .generate((dn, cn, path_trace), DynTrie::new())
        .0;

    // condition on the rendered observation (both channels)
    let mut constraints = DynTrie::new();
    let depth_observation = trace.data.read::<Depths>("depth_observation").clone();
    let color_observation = trace.data.read::<Colors>("color_observation").clone();
    constraints.observe("depth_observation", Arc::new(depth_observation));
    constraints.observe("color_observation", Arc::new(color_observation.clone()));
    let trace = cube_rgbd_model
        .generate((dn, cn, path_trace), constraints)
        .0;

    let mut orbit_azimuth_pass = AddrMap::new();
    orbit_azimuth_pass.visit("orbit_azimuth");

    let mut orbit_elev_radius_pass = AddrMap::new();
    orbit_elev_radius_pass.visit("orbit_sin_elevation");
    orbit_elev_radius_pass.visit("orbit_radius");

    let mut lookat_jitter_pass = AddrMap::new();
    lookat_jitter_pass.visit("lookat_yaw_jitter");
    lookat_jitter_pass.visit("lookat_pitch_jitter");

    let mut ground_pass = AddrMap::new();
    ground_pass.visit("ground_c0");
    ground_pass.visit("ground_c1");
    ground_pass.visit("ground_c2");
    ground_pass.visit("illum_c0");
    ground_pass.visit("illum_c1");
    ground_pass.visit("illum_c2");

    let mut cube_pass = AddrMap::new();
    cube_pass.visit("cube_u");
    cube_pass.visit("cube_v");

    const NUM_ITERS: usize = 250;
    let start = std::time::Instant::now();
    let renders: Vec<Colors> = InferenceKernel::new(&cube_rgbd_model)
        .regen_mh(&orbit_azimuth_pass)
        .regen_mh(&orbit_elev_radius_pass)
        .regen_mh(&lookat_jitter_pass)
        .regen_mh(&ground_pass)
        .regen_mh(&cube_pass)
        .mh(&rgbd_drift, (pass_of(&["cube_u", "cube_v"]), 0.05))
        .iter(trace)
        .take(NUM_ITERS)
        .enumerate()
        .inspect(|(i, _)| {
            println!(
                "iter {}/{NUM_ITERS} ({:.2?} elapsed, {:.3?}/iter)",
                i + 1,
                start.elapsed(),
                start.elapsed() / (*i as u32 + 1)
            )
        })
        .map(|(_, t)| t.retv.clone().unwrap().1)
        .collect();

    let observations = vec![color_observation; NUM_ITERS];
    save_colors2_video("./out/rubiks.mp4", &observations, &renders, 20);
}

#[test]
fn test_derender_ball() {
    create_dir_all("out").expect("error creating 'out' dir");

    let observation = load_colors("./tests/ball.bmp");

    // generate trace
    let mut constraints = DynTrie::new();
    constraints.observe("observation", Arc::new(observation.clone()));
    let trace = ball_model.generate((), constraints).0;

    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam_y");
    cam_pass.visit("cam_yaw");

    let mut env_pass = AddrMap::new();
    env_pass.visit("table_c0");
    env_pass.visit("table_c1");
    env_pass.visit("table_c2");
    env_pass.visit("ambient_brightness");

    let mut ball_pass = AddrMap::new();
    ball_pass.visit("ball_u");
    ball_pass.visit("ball_v");
    ball_pass.visit("ball_radius");

    let mut ball_color_pass = AddrMap::new();
    ball_color_pass.visit("ball_c0");
    ball_color_pass.visit("ball_c1");
    ball_color_pass.visit("ball_c2");

    const NUM_ITERS: usize = 250;
    let renders: Vec<Colors> = InferenceKernel::new(&ball_model)
        .regen_mh(&cam_pass)
        .regen_mh(&env_pass)
        .mh(
            &gaussian_drift,
            (pass_of(&["table_c0", "table_c1", "table_c2"]), 0.1),
        )
        .regen_mh(&ball_pass)
        .mh(
            &gaussian_drift,
            (pass_of(&["ball_u", "ball_v", "ball_radius"]), 0.1),
        )
        .regen_mh(&ball_color_pass)
        .iter(trace)
        .take(NUM_ITERS)
        .map(|t| t.retv.clone().unwrap())
        .collect();

    let observations = vec![observation; NUM_ITERS];
    save_colors2_video("./out/ball.mp4", &observations, &renders, 20);
}
