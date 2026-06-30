use modppl::prelude::*;
use modppl_derender::{
    core::*,
    config::{W, H, AREA, NEAR, FAR, FOVY},
    image::*,
    serialization::*,
    inference::Kernel,
};
use std::fs::create_dir_all;
use glam::{Mat4, Affine3A, Vec3A};


/// quick visual sanity check (no inference) that the Rubik's-cube face coloring
/// looks right: a single still render of the cube sitting on the table.
#[test]
fn test_render_rubiks_cube() {
    create_dir_all("out").expect("error creating 'out' dir");

    let x = Affine3A::from_rotation_translation(
        glam::Quat::from_euler(glam::EulerRot::XYZ, -0.4, 0.6, 0.0),
        [0.0, 1.2, 1.2].into()
    );
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32/H() as f32, NEAR(), FAR());

    let table = (
        Box::new(Plane { origin: Vec3A::ZERO, normal: [0.0, 1.0, 0.0].into() }) as Box<dyn Solid>,
        [0.6, 0.6, 0.6]
    );
    let cube = (
        Box::new(Cube { center: [0.0, 0.4, -0.5].into(), half_extent: 0.4 }) as Box<dyn Solid>,
        [0.0, 0.0, 0.0] // ignored: Cube::color_at overrides per-face
    );

    let mut pixels = vec![[0.0; 3]; AREA()];
    raytrace_colors(x, proj, &vec![table, cube], [0.9, 0.9, 0.9], &mut pixels);
    save_colors("./out/rubiks_cube.bmp", &pixels);
}

#[test]
fn test_derender_ground_depth() {
    create_dir_all("out").expect("error creating 'out' dir");

    // simulate constraints
    let mut synth_constraints = DynTrie::new();
    synth_constraints.observe("cam_roll", Arc::new(0.));
    synth_constraints.observe("cam_y", Arc::new(1.5));
    let trace = grounded_depth_model.generate((), synth_constraints).0;

    // generate trace
    let mut constraints = DynTrie::new();
    let observation = trace.data.read::<Depths>("observation").clone();
    constraints.observe("observation", Arc::new(observation.clone()));
    let trace = grounded_depth_model.generate((), constraints).0;

    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_roll");
    cam_mask.visit("cam_y");

    const NUM_ITERS: usize = 200;
    let renders: Vec<Depths> = Kernel::new(&grounded_depth_model, trace)
        .regen_mh(&cam_mask)
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
        synth_constraints.observe("cam_roll", Arc::new(0.));
        synth_constraints.observe("ground_albedo", Arc::new(0.5));
        synth_constraints.observe("ambient_brightness", Arc::new(0.95));
        let trace = sphere_color_model.generate((), synth_constraints).0;

        // generate trace
        let mut constraints = DynTrie::new();
        let observation = trace.data.read::<Colors>("observation").clone();
        constraints.observe("observation", Arc::new(observation.clone()));
        let trace = sphere_color_model.generate((), constraints).0;

        let mut cam_mask = AddrMap::new();
        cam_mask.visit("cam_y");
        cam_mask.visit("cam_roll");

        let mut pos_mask = AddrMap::new();
        pos_mask.visit("sphere_u");
        pos_mask.visit("sphere_v");

        let mut env_mask = AddrMap::new();
        env_mask.visit("ground_albedo");
        env_mask.visit("ambient_brightness");

        let mut sphere_color_mask = AddrMap::new();
        sphere_color_mask.visit("sphere_redness");

        const NUM_ITERS: usize = 200;
        let renders: Vec<Colors> = Kernel::new(&sphere_color_model, trace)
            .regen_mh(&cam_mask)
            .regen_mh(&pos_mask)
            .regen_mh(&pos_mask)
            .regen_mh(&pos_mask)
            .regen_mh(&env_mask)
            .regen_mh(&sphere_color_mask)
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
    synth_constraints.observe("cam_yaw", Arc::new(0.0));
    let trace = mug_model.generate(MUG_NOISE, synth_constraints).0;

    // condition on the rendered observation
    let mut constraints = DynTrie::new();
    let observation = trace.data.read::<Colors>("observation").clone();
    constraints.observe("observation", Arc::new(observation.clone()));
    let trace = mug_model.generate(MUG_NOISE, constraints).0;

    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_y");
    cam_mask.visit("cam_yaw");

    let mut env_mask = AddrMap::new();
    env_mask.visit("table_c0");
    env_mask.visit("table_c1");
    env_mask.visit("table_c2");
    env_mask.visit("ambient_brightness");

    let mut mug_mask = AddrMap::new();
    mug_mask.visit("mug_u");
    mug_mask.visit("mug_v");
    mug_mask.visit("mug_radius");
    mug_mask.visit("mug_height");

    let mut mug_color_mask = AddrMap::new();
    mug_color_mask.visit("mug_c0");
    mug_color_mask.visit("mug_c1");
    mug_color_mask.visit("mug_c2");

    const NUM_ITERS: usize = 250;
    let start = std::time::Instant::now();
    let renders: Vec<Colors> = Kernel::new(&mug_model, trace)
        .regen_mh(&cam_mask)
        .regen_mh(&env_mask)
        .regen_mh(&mug_mask)
        .mh(&noise_drift, (vec!["mug_u", "mug_v", "mug_radius", "mug_height"], 0.1))
        .regen_mh(&mug_color_mask)
        .take(NUM_ITERS)
        .enumerate()
        .inspect(|(i, _)| println!(
            "iter {}/{NUM_ITERS} ({:.2?} elapsed, {:.3?}/iter)",
            i + 1, start.elapsed(), start.elapsed() / (*i as u32 + 1)
        ))
        .map(|(_, t)| t.retv.clone().unwrap())
        .collect();

    let observations = vec![observation; NUM_ITERS];
    save_colors2_video("./out/mug.mp4", &observations, &renders, 20);
}

// fixed observation noise, no annealing: same reasoning as the mug model
// (single object, unimodal posterior).
const RUBIKS_NOISE: f32 = 0.05;

#[test]
fn test_derender_rubiks() {
    create_dir_all("out").expect("error creating 'out' dir");

    // simulate constraints (fix the camera yaw so the cube stays in frame)
    let mut synth_constraints = DynTrie::new();
    synth_constraints.observe("cam_yaw", Arc::new(0.0));
    let trace = rubiks_model.generate(RUBIKS_NOISE, synth_constraints).0;

    // condition on the rendered observation
    let mut constraints = DynTrie::new();
    let observation = trace.data.read::<Colors>("observation").clone();
    constraints.observe("observation", Arc::new(observation.clone()));
    let trace = rubiks_model.generate(RUBIKS_NOISE, constraints).0;

    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_y");
    cam_mask.visit("cam_yaw");

    let mut env_mask = AddrMap::new();
    env_mask.visit("table_c0");
    env_mask.visit("table_c1");
    env_mask.visit("table_c2");
    env_mask.visit("ambient_brightness");

    let mut cube_mask = AddrMap::new();
    cube_mask.visit("cube_u");
    cube_mask.visit("cube_v");
    cube_mask.visit("cube_size");

    const NUM_ITERS: usize = 250;
    let start = std::time::Instant::now();
    let renders: Vec<Colors> = Kernel::new(&rubiks_model, trace)
        .regen_mh(&cam_mask)
        .regen_mh(&env_mask)
        .regen_mh(&cube_mask)
        .mh(&noise_drift, (vec!["cube_u", "cube_v", "cube_size"], 0.1))
        .take(NUM_ITERS)
        .enumerate()
        .inspect(|(i, _)| println!(
            "iter {}/{NUM_ITERS} ({:.2?} elapsed, {:.3?}/iter)",
            i + 1, start.elapsed(), start.elapsed() / (*i as u32 + 1)
        ))
        .map(|(_, t)| t.retv.clone().unwrap())
        .collect();

    let observations = vec![observation; NUM_ITERS];
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

    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_y");
    cam_mask.visit("cam_yaw");

    let mut env_mask = AddrMap::new();
    env_mask.visit("table_c0");
    env_mask.visit("table_c1");
    env_mask.visit("table_c2");
    env_mask.visit("ambient_brightness");

    let mut ball_mask = AddrMap::new();
    ball_mask.visit("ball_u");
    ball_mask.visit("ball_v");
    ball_mask.visit("ball_radius");

    let mut ball_color_mask = AddrMap::new();
    ball_color_mask.visit("ball_c0");
    ball_color_mask.visit("ball_c1");
    ball_color_mask.visit("ball_c2");

    const NUM_ITERS: usize = 250;
    let renders: Vec<Colors> = Kernel::new(&ball_model, trace)
        .regen_mh(&cam_mask)
        .regen_mh(&env_mask)
        .mh(&gaussian_drift, (vec!["table_c0", "table_c1", "table_c2"], 0.1))
        .regen_mh(&ball_mask)
        .mh(&gaussian_drift, (vec!["ball_u", "ball_v", "ball_radius"], 0.1))
        .regen_mh(&ball_color_mask)
        .take(NUM_ITERS)
        .map(|t| t.retv.clone().unwrap())
        .collect();

    let observations = vec![observation; NUM_ITERS];
    save_colors2_video("./out/ball.mp4", &observations, &renders, 20);
}