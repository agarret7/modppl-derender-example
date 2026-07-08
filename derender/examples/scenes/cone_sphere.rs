//! Live-window demo of the library's `cone_sphere_model` (two objects
//! floating against the background, no ground plane: a purple cone and a
//! red sphere, both hand-positioned via constrained observation -- see the
//! comments below).
//!
//! Run it:
//!   cargo run --release --example sandbox -- cone_sphere
//! Keys: Space = pause, R = resample, S = snapshot, ESC = quit.

#![allow(non_upper_case_globals)]

use std::f32::consts::PI;

use glam::{Affine3A, EulerRot, Mat4, Quat};
use modppl::prelude::*;
use modppl_derender::{
    config::{AREA, FAR, FOVY, H, NEAR, W},
    core::*,
    image::Colors,
    inference::InferenceKernel,
    sandbox::{run_sandbox_loop, Panels},
};

// ─── sub-generative functions, one per object ────────────────────────────────
// mirrors the InferenceKernel's own grouping below (one move-group per object): each
// `/=` call here is a compound address, so a pass can select the whole group
// at once (`.visit("cam")`) or drill into one latent (`.visit("cam/y")`)
// -- see AddrMap::search / DynGenFnHandler::trace_at for how that dispatches.
// Sub-models return plain numeric tuples, not `Box<dyn Solid>` directly: a
// `/=` callee's return type must be Clone, and trait objects aren't.

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
    /// shared by both objects -- their (u, v) priors happen to be identical,
    /// so this is one compound choice ("pos" -> u, v) instead of two loose
    /// `%=` lines repeated in every object's params model.
    fn position_model() -> (f32, f32) {
        let u = uniform(-1.0, 1.0) %= "u";
        let v = uniform(-0.7, 0.8) %= "v";
        (u, v)
    }
);

dyngen!(
    /// shared by both objects' colors -- same pattern as tutorial_04's
    /// `uniform_color`: one compound choice instead of three loose `%=` lines.
    fn uniform_color(lb: f32, ub: f32) -> [f32; 3] {
        let c0 = uniform(lb, ub) %= "c0";
        let c1 = uniform(lb, ub) %= "c1";
        let c2 = uniform(lb, ub) %= "c2";
        [c0, c1, c2]
    }
);

dyngen!(
    /// apex up, with unknown position, size, and color (floats against the
    /// background -- there is no ground plane in this scene).
    fn cone_params_model() -> (f32, f32, f32, f32, [f32; 3]) {
        let (cone_u, cone_v) = position_model() /= "pos";
        let cone_height = uniform(0.3, 0.8) %= "height";
        let cone_radius = uniform(0.15, 0.4) %= "radius";
        let cone_c = uniform_color(0.25, 1.0) /= "color";
        (cone_u, cone_v, cone_height, cone_radius, cone_c)
    }
);

dyngen!(
    /// unknown position, size, and color.
    fn sphere_params_model() -> (f32, f32, f32, [f32; 3]) {
        let (sphere_u, sphere_v) = position_model() /= "pos";
        let sphere_radius = uniform(0.15, 0.35) %= "radius";
        let sphere_c = uniform_color(0.25, 1.0) /= "color";
        (sphere_u, sphere_v, sphere_radius, sphere_c)
    }
);

dyngen!(
    /// two independent objects on a shared table: a cone and a sphere, each with
    /// its own position, size, and color.
    fn cone_sphere_model(noise: f32) -> Colors {
        let x = camera_model() /= "cam";

        // background
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

        // render: full path tracer -- both objects are plain-colored (no fixed
        // per-face scheme to preserve, unlike the Rubik's cube), so realistic
        // diffuse shading and shadows only help sim-to-real color matching here.
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
        let mut pixels = vec![[0.0; 3]; AREA()];
        raytrace_colors(x, proj, &vec![cone, sphere], background_c, &mut pixels);
        noisy_colors(pixels.clone(), noise) %= "observation";

        pixels
    }
);

/// probability of a full prior jump (vs. a local drift step) for a block of
/// latents: mostly drift for smooth refinement, occasionally a global jump so
/// the chain can escape a bad lock.
const JUMP_PROB: f32 = 0.15;

/// coarse-to-fine drift ladder, applied per object per sweep: the coarse
/// scale relocates a mislocked object wholesale (big enough to carry it off
/// one blob and onto another), the mid scale settles it, the fine scale
/// polishes. Every rung is a full MH move, so bad coarse proposals just get
/// rejected -- the ladder costs renders, not correctness.
const DRIFT_SCALES: [f32; 3] = [0.2, 0.08, 0.02];

// ─── move profiling ──────────────────────────────────────────────────────────
// per-move acceptance counters, printed live under the window. This is how
// you find out WHICH move is failing: a 0% coarse-drift rung and a healthy
// fine rung means the chain is stuck polishing a local mode, etc.

use std::sync::atomic::{AtomicU32, Ordering};

struct MoveStat {
    name: &'static str,
    attempts: AtomicU32,
    accepts: AtomicU32,
}

const fn stat(name: &'static str) -> MoveStat {
    MoveStat {
        name,
        attempts: AtomicU32::new(0),
        accepts: AtomicU32::new(0),
    }
}

static STATS: [MoveStat; 15] = [
    stat("cam jump"),           // 0
    stat("cam drift"),          // 1
    stat("env regen"),          // 2
    stat("cone pos jump"),      // 3
    stat("cone drift 0.20"),    // 4
    stat("cone drift 0.08"),    // 5
    stat("cone drift 0.02"),    // 6
    stat("cone color jump"),    // 7
    stat("cone color drift"),   // 8
    stat("sphere pos jump"),    // 9
    stat("sphere drift 0.20"),  // 10
    stat("sphere drift 0.08"),  // 11
    stat("sphere drift 0.02"),  // 12
    stat("sphere color jump"),  // 13
    stat("sphere color drift"), // 14
];

fn tally<T>(idx: usize, r: (T, bool)) -> (T, bool) {
    STATS[idx].attempts.fetch_add(1, Ordering::Relaxed);
    if r.1 {
        STATS[idx].accepts.fetch_add(1, Ordering::Relaxed);
    }
    r
}

fn stats_string() -> String {
    STATS
        .iter()
        .map(|s| {
            let att = s.attempts.load(Ordering::Relaxed);
            let acc = s.accepts.load(Ordering::Relaxed);
            let pct = if att > 0 {
                100.0 * acc as f32 / att as f32
            } else {
                0.0
            };
            format!("{:<19} {:>6}/{:<6} {:>5.1}%\n", s.name, acc, att, pct)
        })
        .collect()
}

pub fn run() {
    // hoisted above run_sandbox_loop: the InferenceKernel-builder closures below
    // borrow these (not move), with a lifetime tied to this run() call --
    // that's fine since the built `kernel` (which holds those borrows) is
    // itself what gets moved into the final per-frame stepper closure, and
    // everything lives within this single invocation.
    // visiting the compound address as a bare leaf (no further path) means
    // "regenerate everything under here" -- see AddrMap::search plus the
    // is_leaf() shorthand in modppl's regenerate entrypoint.
    let mut cam_pass = AddrMap::new();
    cam_pass.visit("cam");

    let cam_roll_pass = pass_of(&["cam/roll"]);

    let mut env_pass = AddrMap::new();
    env_pass.visit("ambient_brightness");

    // position-only jump blocks: a prior resample of 2 latents gets accepted
    // far more often than one that must land position AND shape together.
    // "cone/pos" is itself a compound ("pos" -> u, v), so visiting it as a
    // bare leaf resamples both together -- exactly the old two-visit behavior.
    let mut cone_pos_pass = AddrMap::new();
    cone_pos_pass.visit("cone/pos");

    let mut cone_color_pass = AddrMap::new();
    cone_color_pass.visit("cone/color");

    let mut sphere_pos_pass = AddrMap::new();
    sphere_pos_pass.visit("sphere/pos");

    let mut sphere_color_pass = AddrMap::new();
    sphere_color_pass.visit("sphere/color");

    run_sandbox_loop(
        "derender: cone+sphere  |  obs : hyp  |  Space=pause  R=resample  S=save  ESC=quit",
        || {
            // fresh chain, fresh acceptance stats
            for s in STATS.iter() {
                s.attempts.store(0, Ordering::Relaxed);
                s.accepts.store(0, Ordering::Relaxed);
            }

            // constrain the *observation*, not the model: these fix the
            // ground-truth scene used to synthesize the observed image
            let mut synth = DynTrie::new();
            synth.observe("cam/yaw", Arc::new(0.0_f32));
            synth.observe("cam/roll", Arc::new(0.0_f32));
            synth.observe("cam/y", Arc::new(0.2_f32));
            synth.observe("sphere/radius", Arc::new(0.15_f32));
            synth.observe("cone/radius", Arc::new(0.225_f32)); // 1.5x the ball's radius
            synth.observe("cone/height", Arc::new(0.75_f32));
            synth.observe("sphere/pos/u", Arc::new(0.15_f32));
            synth.observe("sphere/pos/v", Arc::new(0.6_f32));
            synth.observe("cone/pos/u", Arc::new(-0.05_f32));
            synth.observe("cone/pos/v", Arc::new(0.3_f32));
            synth.observe("cone/color/c0", Arc::new(0.5_f32)); // B: darker purple
            synth.observe("cone/color/c1", Arc::new(0.25_f32)); // G (at the prior's floor)
            synth.observe("cone/color/c2", Arc::new(0.5_f32)); // R
            synth.observe("sphere/color/c0", Arc::new(0.25_f32)); // B: red
            synth.observe("sphere/color/c1", Arc::new(0.25_f32)); // G
            synth.observe("sphere/color/c2", Arc::new(1.0_f32)); // R
            const NOISE: f32 = 0.05;
            let gt = cone_sphere_model.generate(NOISE, synth).0;
            let observation = gt.data.read::<Colors>("observation").clone();

            let mut constraints = DynTrie::new();
            constraints.observe("observation", Arc::new(observation.clone()));
            let trace = cone_sphere_model.generate(5.0 * NOISE, constraints).0;

            // camera: local drift most sweeps, occasional prior jump.
            // per object: an occasional position-only prior jump to find (or
            // re-find) it, then the full coarse-to-fine drift ladder over
            // position+shape -- the coarse rung is what lets a mislocked
            // object relocate wholesale instead of being stuck polishing the
            // wrong blob. Colors drift, with occasional jumps.
            let kernel = InferenceKernel::new(&cone_sphere_model)
                .then(|t| {
                    let mut rng = ThreadRng::default();
                    if u01(&mut rng) < JUMP_PROB {
                        tally(0, regen_mh(&cone_sphere_model, t, &cam_pass))
                    } else {
                        tally(
                            1,
                            mh(
                                &cone_sphere_model,
                                t,
                                &noise_drift,
                                (pass_of(&["cam/y", "cam/yaw", "cam/roll"]), 0.03),
                            ),
                        )
                    }
                })
                .mh(&noise_drift, (cam_roll_pass.clone(), 0.05))
                .then(|t| tally(2, regen_mh(&cone_sphere_model, t, &env_pass)))
                .then(|t| {
                    let mut rng = ThreadRng::default();
                    if u01(&mut rng) < JUMP_PROB {
                        tally(3, regen_mh(&cone_sphere_model, t, &cone_pos_pass))
                    } else {
                        (t, false)
                    }
                })
                .then(|t| {
                    let addrs = pass_of(&["cone/pos/u", "cone/pos/v", "cone/height", "cone/radius"]);
                    let t = tally(
                        4,
                        mh(
                            &cone_sphere_model,
                            t,
                            &noise_drift,
                            (addrs.clone(), DRIFT_SCALES[0]),
                        ),
                    )
                    .0;
                    let t = tally(
                        5,
                        mh(
                            &cone_sphere_model,
                            t,
                            &noise_drift,
                            (addrs.clone(), DRIFT_SCALES[1]),
                        ),
                    )
                    .0;
                    tally(
                        6,
                        mh(
                            &cone_sphere_model,
                            t,
                            &noise_drift,
                            (addrs, DRIFT_SCALES[2]),
                        ),
                    )
                })
                .then(|t| {
                    let mut rng = ThreadRng::default();
                    if u01(&mut rng) < JUMP_PROB {
                        tally(7, regen_mh(&cone_sphere_model, t, &cone_color_pass))
                    } else {
                        tally(
                            8,
                            mh(
                                &cone_sphere_model,
                                t,
                                &noise_drift,
                                (pass_of(&["cone/color/c0", "cone/color/c1", "cone/color/c2"]), 0.05),
                            ),
                        )
                    }
                })
                .then(|t| {
                    let mut rng = ThreadRng::default();
                    if u01(&mut rng) < JUMP_PROB {
                        tally(9, regen_mh(&cone_sphere_model, t, &sphere_pos_pass))
                    } else {
                        (t, false)
                    }
                })
                .then(|t| {
                    let addrs = pass_of(&["sphere/pos/u", "sphere/pos/v", "sphere/radius"]);
                    let t = tally(
                        10,
                        mh(
                            &cone_sphere_model,
                            t,
                            &noise_drift,
                            (addrs.clone(), DRIFT_SCALES[0]),
                        ),
                    )
                    .0;
                    let t = tally(
                        11,
                        mh(
                            &cone_sphere_model,
                            t,
                            &noise_drift,
                            (addrs.clone(), DRIFT_SCALES[1]),
                        ),
                    )
                    .0;
                    tally(
                        12,
                        mh(
                            &cone_sphere_model,
                            t,
                            &noise_drift,
                            (addrs, DRIFT_SCALES[2]),
                        ),
                    )
                })
                .then(|t| {
                    let mut rng = ThreadRng::default();
                    if u01(&mut rng) < JUMP_PROB {
                        tally(13, regen_mh(&cone_sphere_model, t, &sphere_color_pass))
                    } else {
                        tally(
                            14,
                            mh(
                                &cone_sphere_model,
                                t,
                                &noise_drift,
                                (pass_of(&["sphere/color/c0", "sphere/color/c1", "sphere/color/c2"]), 0.05),
                            ),
                        )
                    }
                });

            let mut printed_lines = 0usize;
            let mut trace = trace;
            move |_keys: &[minifb::Key]| {
                trace = kernel.step(trace.clone());
                let text = format!("logjp {:>10.1}\n{}", trace.logjp, stats_string());
                if printed_lines > 0 {
                    print!("\x1B[{printed_lines}A\x1B[0J");
                }
                print!("{text}");
                use std::io::Write;
                std::io::stdout().flush().unwrap();
                printed_lines = text.lines().count();
                Panels::Color {
                    obs: observation.clone(),
                    hyp: trace.retv.clone().unwrap(),
                }
            }
        },
    )
    .expect("window error");
}
