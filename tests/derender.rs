use modppl::prelude::*;
use modppl_derender::*;
use std::fs::create_dir_all;


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
    let mut trace = grounded_depth_model.generate((), constraints).0;

    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_y");
    cam_mask.visit("cam_roll");

    const NUM_ITERS: usize = 200;
    let mut renders = vec![];
    for _ in 0..NUM_ITERS {
        let (new_trace, _) = regen_mh(&grounded_depth_model, trace, &cam_mask);
        trace = new_trace;
        renders.push(trace.retv.clone().unwrap());
    }

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
        let mut trace = sphere_color_model.generate((), constraints).0;

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
        let mut renders = vec![];
        for _ in 0..NUM_ITERS {
            let (new_trace, _) = regen_mh(&sphere_color_model, trace, &cam_mask);
            trace = new_trace;
            for _ in 0..3 {
                let (new_trace, _) = regen_mh(&sphere_color_model, trace, &pos_mask);
                trace = new_trace;
            }
            let (new_trace, _) = regen_mh(&sphere_color_model, trace, &env_mask);
            trace = new_trace;
            let (new_trace, _) = regen_mh(&sphere_color_model, trace, &sphere_color_mask);
            trace = new_trace;
            renders.push(trace.retv.clone().unwrap());
        }

        let observations = vec![observation; NUM_ITERS];
        save_colors2_video(&format!("./out/sphere{i}.mp4"), &observations, &renders, 20);
    }
}

#[test]
fn test_derender_ball() {
    create_dir_all("out").expect("error creating 'out' dir");

    let observation = load_colors("./tests/ball.bmp");

    // generate trace
    let mut constraints = DynTrie::new();
    constraints.observe("observation", Arc::new(observation.clone()));
    let mut trace = ball_model.generate((), constraints).0;

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
    let mut renders = vec![];
    for _ in 0..NUM_ITERS {
        let (new_trace, _) = regen_mh(&ball_model, trace, &cam_mask);
        trace = new_trace;
        let (new_trace, _) = regen_mh(&ball_model, trace, &env_mask);
        trace = new_trace;
        let (new_trace, _) = mh(&ball_model, trace, &gaussian_drift, (vec!["table_c0", "table_c1", "table_c2"], 0.1));
        trace = new_trace;
        let (new_trace, _) = regen_mh(&ball_model, trace, &ball_mask);
        trace = new_trace;
        let (new_trace, _) = mh(&ball_model, trace, &gaussian_drift, (vec!["ball_u", "ball_v", "ball_radius"], 0.1));
        trace = new_trace;
        let (new_trace, _) = regen_mh(&ball_model, trace, &ball_color_mask);
        trace = new_trace;
        renders.push(trace.retv.clone().unwrap());
    }

    let observations = vec![observation; NUM_ITERS];
    save_colors2_video("./out/ball.mp4", &observations, &renders, 20);
}