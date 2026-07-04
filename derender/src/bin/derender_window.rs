//! Live-window synthetic derender demos, all running on the model-independent
//! `sandbox` harness: each model contributes only a `restart` closure (sample
//! ground truth, condition, build a Kernel) returning a stepper (one sweep per
//! frame, say what to display). Window management, pause (Space), restart (R),
//! snapshot (S), and quit (ESC) live in the harness.
//!
//! Usage:
//!   cargo run --release --bin derender_window [ground|sphere|mug|cone_sphere|rubiks]
//!
//! Resolution via the RES env var (default 64):
//!   RES=128 cargo run --release --bin derender_window sphere
//! The rubiks mode uses `cube_rgbd_model` and should run in the cube pipeline
//! environment (see README).

use modppl::prelude::*;
use modppl_derender::{
    core::*,
    image::{Colors, Depths},
    inference::Kernel,
    sandbox::{run_sandbox_loop, Panels},
};

fn run_ground() {
    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_roll");
    cam_mask.visit("cam_y");

    run_sandbox_loop(
        "derender: ground  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let mut synth = DynTrie::new();
            synth.observe("cam_roll", Arc::new(0.0_f32));
            synth.observe("cam_y",    Arc::new(1.5_f32));
            let gt          = grounded_depth_model.generate((), synth).0;
            let observation = gt.data.read::<Depths>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone()));
            let trace = grounded_depth_model.generate((), constraints).0;

            let mut kernel = Kernel::new(&grounded_depth_model, trace)
                .regen_mh(&cam_mask);

            move |_keys: &[minifb::Key]| {
                let t = kernel.next().unwrap();
                Panels::Depth { obs: observation.clone(), hyp: t.retv.clone().unwrap() }
            }
        },
    ).expect("window error");
}

fn run_sphere() {
    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_y");
    cam_mask.visit("cam_roll");

    let mut pos_mask = AddrMap::new();
    pos_mask.visit("sphere_u");
    pos_mask.visit("sphere_v");

    let mut env_mask = AddrMap::new();
    env_mask.visit("ground_albedo");
    env_mask.visit("ambient_brightness");

    let mut color_mask = AddrMap::new();
    color_mask.visit("sphere_redness");

    run_sandbox_loop(
        "derender: sphere  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let mut synth = DynTrie::new();
            synth.observe("cam_roll",           Arc::new(0.0_f32));
            synth.observe("ground_albedo",      Arc::new(0.5_f32));
            synth.observe("ambient_brightness", Arc::new(0.95_f32));
            let gt          = sphere_color_model.generate((), synth).0;
            let observation = gt.data.read::<Colors>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone()));
            let trace = sphere_color_model.generate((), constraints).0;

            let mut kernel = Kernel::new(&sphere_color_model, trace)
                .regen_mh(&cam_mask)
                .regen_mh(&pos_mask)
                .regen_mh(&pos_mask)
                .regen_mh(&pos_mask)
                .regen_mh(&env_mask)
                .regen_mh(&color_mask);

            move |_keys: &[minifb::Key]| {
                let t = kernel.next().unwrap();
                Panels::Color { obs: observation.clone(), hyp: t.retv.clone().unwrap() }
            }
        },
    ).expect("window error");
}

fn run_mug() {
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

    let mut color_mask = AddrMap::new();
    color_mask.visit("mug_c0");
    color_mask.visit("mug_c1");
    color_mask.visit("mug_c2");

    run_sandbox_loop(
        "derender: mug  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            let mut synth = DynTrie::new();
            synth.observe("cam_yaw", Arc::new(0.0_f32));
            let gt          = mug_model.generate(0.05_f32, synth).0;
            let observation = gt.data.read::<Colors>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone()));
            let trace = mug_model.generate(0.05_f32, constraints).0;

            let mut kernel = Kernel::new(&mug_model, trace)
                .regen_mh(&cam_mask)
                .regen_mh(&env_mask)
                .regen_mh(&mug_mask)
                .mh(&noise_drift, (vec!["mug_u", "mug_v", "mug_radius", "mug_height"], 0.1))
                .regen_mh(&color_mask);

            move |_keys: &[minifb::Key]| {
                let t = kernel.next().unwrap();
                Panels::Color { obs: observation.clone(), hyp: t.retv.clone().unwrap() }
            }
        },
    ).expect("window error");
}

fn run_cone_sphere() {
    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_y");
    cam_mask.visit("cam_yaw");
    cam_mask.visit("cam_roll");

    let mut env_mask = AddrMap::new();
    env_mask.visit("table_c0");
    env_mask.visit("table_c1");
    env_mask.visit("table_c2");
    env_mask.visit("ambient_brightness");

    let mut cone_mask = AddrMap::new();
    cone_mask.visit("cone_u");
    cone_mask.visit("cone_v");
    cone_mask.visit("cone_height");
    cone_mask.visit("cone_radius");

    let mut cone_color_mask = AddrMap::new();
    cone_color_mask.visit("cone_c0");
    cone_color_mask.visit("cone_c1");
    cone_color_mask.visit("cone_c2");

    let mut sphere_mask = AddrMap::new();
    sphere_mask.visit("sphere_u");
    sphere_mask.visit("sphere_v");
    sphere_mask.visit("sphere_radius");

    let mut sphere_color_mask = AddrMap::new();
    sphere_color_mask.visit("sphere_c0");
    sphere_color_mask.visit("sphere_c1");
    sphere_color_mask.visit("sphere_c2");

    run_sandbox_loop(
        "derender: cone+sphere  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            // constrain the *observation*, not the model: these fix the
            // ground-truth scene used to synthesize the observed image, but
            // inference still explores the full, unconstrained prior below --
            // nothing here changes what the chain is allowed to propose.
            //
            // Colors are BGR (see image.rs); the model's per-channel color
            // prior is Uniform(0.25, 1.0), so a channel can't go all the way
            // to 0 -- "off" channels are pinned at the prior's floor (0.25)
            // instead, which still reads clearly as red/purple.
            // cone_height ~ U(0.3,0.8) and sphere_radius ~ U(0.15,0.35) only
            // barely overlap for a 2.5x ratio; sphere_radius at its own
            // floor (0.15) gives cone_height = 0.75, safely inside its range
            // -- both endpoints stay in-prior so inference can converge onto
            // them (a true value outside the prior's support could never be
            // recovered by regen_mh, which only ever proposes from the prior).
            // cam_yaw=0 points the camera straight down -z, so world x=0 is
            // dead-center in the frame; v is depth (more negative = farther
            // from the camera at z=1.2, i.e. farther back in the scene).
            let mut synth = DynTrie::new();
            synth.observe("cam_yaw", Arc::new(0.0_f32));
            synth.observe("cam_roll", Arc::new(0.0_f32));
            synth.observe("cam_y", Arc::new(0.2_f32));
            synth.observe("sphere_radius", Arc::new(0.15_f32));
            synth.observe("cone_radius", Arc::new(0.225_f32)); // 1.5x the ball's radius
            synth.observe("cone_height", Arc::new(0.75_f32));
            synth.observe("sphere_u", Arc::new(0.0_f32));   // dead-center
            synth.observe("sphere_v", Arc::new(-0.15_f32));
            synth.observe("cone_u", Arc::new(-0.2_f32));    // a bit left of the sphere
            synth.observe("cone_v", Arc::new(-0.3_f32));    // and a bit farther back
            synth.observe("cone_c0", Arc::new(0.5_f32));  // B: darker purple
            synth.observe("cone_c1", Arc::new(0.25_f32)); // G (at the prior's floor)
            synth.observe("cone_c2", Arc::new(0.5_f32));  // R
            synth.observe("sphere_c0", Arc::new(0.25_f32)); // B: red
            synth.observe("sphere_c1", Arc::new(0.25_f32)); // G
            synth.observe("sphere_c2", Arc::new(1.0_f32));  // R
            synth.observe("table_c0", Arc::new(0.9_f32)); // whitish ground
            synth.observe("table_c1", Arc::new(0.9_f32));
            synth.observe("table_c2", Arc::new(0.9_f32));
            let gt          = cone_sphere_model.generate(0.05_f32, synth).0;
            let observation = gt.data.read::<Colors>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone()));
            let trace = cone_sphere_model.generate(0.05_f32, constraints).0;

            let mut kernel = Kernel::new(&cone_sphere_model, trace)
                .regen_mh(&cam_mask)
                .regen_mh(&env_mask)
                .regen_mh(&cone_mask)
                .mh(&noise_drift, (vec!["cone_u", "cone_v", "cone_height", "cone_radius"], 0.1))
                .regen_mh(&cone_color_mask)
                .regen_mh(&sphere_mask)
                .mh(&noise_drift, (vec!["sphere_u", "sphere_v", "sphere_radius"], 0.1))
                .regen_mh(&sphere_color_mask);

            move |_keys: &[minifb::Key]| {
                let t = kernel.next().unwrap();
                Panels::Color { obs: observation.clone(), hyp: t.retv.clone().unwrap() }
            }
        },
    ).expect("window error");
}

fn run_rubiks() {
    // the one cube model, end-to-end -- path-traced for visual appeal, and
    // displayed as the full 2x2 RGB-D grid since the model observes both.
    modppl_derender::config::apply_cube_pipeline_defaults();
    const NOISE: (f32, f32, bool) = (0.05, 0.05, true);

    let mut orbit_azimuth_mask = AddrMap::new();
    orbit_azimuth_mask.visit("orbit_azimuth");

    let mut orbit_elev_radius_mask = AddrMap::new();
    orbit_elev_radius_mask.visit("orbit_sin_elevation");
    orbit_elev_radius_mask.visit("orbit_radius");

    let mut lookat_jitter_mask = AddrMap::new();
    lookat_jitter_mask.visit("lookat_yaw_jitter");
    lookat_jitter_mask.visit("lookat_pitch_jitter");

    let mut ground_mask = AddrMap::new();
    ground_mask.visit("ground_c0");
    ground_mask.visit("ground_c1");
    ground_mask.visit("ground_c2");
    ground_mask.visit("illum_c0");
    ground_mask.visit("illum_c1");
    ground_mask.visit("illum_c2");

    let mut cube_mask = AddrMap::new();
    cube_mask.visit("cube_u");
    cube_mask.visit("cube_v");

    run_sandbox_loop(
        "derender: rubiks  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            // orbital camera always points at the cube, no constraints needed
            let gt        = cube_rgbd_model.generate(NOISE, DynTrie::new()).0;
            let obs_depth = gt.data.read::<Depths>("depth_observation").clone();
            let obs_color = gt.data.read::<Colors>("color_observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("depth_observation", Arc::new(obs_depth.clone()));
            constraints.observe("color_observation", Arc::new(obs_color.clone()));
            let trace = cube_rgbd_model.generate(NOISE, constraints).0;

            let mut kernel = Kernel::new(&cube_rgbd_model, trace)
                .regen_mh(&orbit_azimuth_mask)
                .regen_mh(&orbit_elev_radius_mask)
                .regen_mh(&lookat_jitter_mask)
                .regen_mh(&ground_mask)
                .regen_mh(&cube_mask)
                .mh(&rgbd_drift, (vec!["cube_u", "cube_v"], 0.05));

            move |_keys: &[minifb::Key]| {
                let t = kernel.next().unwrap();
                Panels::Rgbd {
                    obs: (obs_depth.clone(), obs_color.clone()),
                    hyp: t.retv.clone().unwrap(),
                }
            }
        },
    ).expect("window error");
}

fn main() {
    let model = std::env::args().nth(1).unwrap_or_else(|| "ground".to_string());
    match model.as_str() {
        "ground"      => run_ground(),
        "sphere"      => run_sphere(),
        "mug"         => run_mug(),
        "cone_sphere" => run_cone_sphere(),
        "rubiks"      => run_rubiks(),
        other    => eprintln!(
            "unknown model '{other}'\nusage: derender_window [ground|sphere|mug|cone_sphere|rubiks]"
        ),
    }
}
