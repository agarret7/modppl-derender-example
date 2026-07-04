use modppl::prelude::*;
use modppl_derender::{
    core::*,
    image::{Colors, Depths},
    inference::Kernel,
    live::run_rgbd_loop,
};

const DEFAULT_DEPTH_NOISE: f32 = 0.2;
const DEFAULT_COLOR_NOISE: f32 = 0.32;
const SWEEPS_PER_FRAME: usize = 5;
const ORBIT_RESAMPLE_PROB: f64 = 0.1;

fn parse_args() -> (f32, f32, bool) {
    let args: Vec<String> = std::env::args().collect();
    let mut depth_noise = DEFAULT_DEPTH_NOISE;
    let mut color_noise = DEFAULT_COLOR_NOISE;
    // flat by default: the path tracer's Monte Carlo sampling makes the
    // likelihood stochastic, which corrupts MH acceptance ratios.
    let mut path_trace = false;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--d-noise" | "-d" => {
                if let Some(v) = args.get(i + 1).and_then(|s| s.parse().ok()) {
                    depth_noise = v; i += 2;
                } else { eprintln!("--d-noise requires a float"); i += 1; }
            }
            "--rgb-noise" | "-c" => {
                if let Some(v) = args.get(i + 1).and_then(|s| s.parse().ok()) {
                    color_noise = v; i += 2;
                } else { eprintln!("--rgb-noise requires a float"); i += 1; }
            }
            "--path-trace" => { path_trace = true; i += 1; }
            _ => { i += 1; }
        }
    }
    (depth_noise, color_noise, path_trace)
}

fn main() -> anyhow::Result<()> {
    modppl_derender::config::apply_cube_pipeline_defaults();
    let (depth_noise, color_noise, path_trace) = parse_args();
    println!(
        "noise: d={depth_noise}  rgb={color_noise}  renderer={}  (ablate color with --rgb-noise 0.99)",
        if path_trace { "path" } else { "flat" }
    );

    // masks defined here so they outlive the closure (Kernel borrows them)
    let mut cube_mask = AddrMap::new();
    cube_mask.visit("cube_u");
    cube_mask.visit("cube_v");

    let mut orbit_azimuth_mask = AddrMap::new();
    orbit_azimuth_mask.visit("orbit_azimuth");

    let mut orbit_radius_elevation_mask = AddrMap::new();
    orbit_radius_elevation_mask.visit("orbit_radius");
    orbit_radius_elevation_mask.visit("orbit_sin_elevation");

    let mut lookat_jitter_mask = AddrMap::new();
    lookat_jitter_mask.visit("lookat_yaw_jitter");
    lookat_jitter_mask.visit("lookat_pitch_jitter");

    let mut ground_albedo_mask = AddrMap::new();
    ground_albedo_mask.visit("ground_c0");
    ground_albedo_mask.visit("ground_c1");
    ground_albedo_mask.visit("ground_c2");
    ground_albedo_mask.visit("illum_c0");
    ground_albedo_mask.visit("illum_c1");
    ground_albedo_mask.visit("illum_c2");

    let mut trace: Option<DynTrace<(f32, f32, bool), (Depths, Colors)>> = None;

    run_rgbd_loop(|obs_depth, obs_color| {
        let mut constraints = DynTrie::new();
        constraints.observe("depth_observation", Arc::new(obs_depth));
        constraints.observe("color_observation", Arc::new(obs_color));

        let current = match trace.take() {
            None => cube_rgbd_model.generate((depth_noise, color_noise, path_trace), constraints).0,
            Some(t) => cube_rgbd_model.update(
                t, (depth_noise, color_noise, path_trace), ArgDiff::NoChange, constraints,
            ).0,
        };

        let result = Kernel::new(&cube_rgbd_model, current)
            .regen_mh(&cube_mask)
            .mh(&rgbd_drift, (vec!["cube_u", "cube_v"], 0.05))
            .then(|t| {
                let mut rng = ThreadRng::default();
                if u01(&mut rng) < ORBIT_RESAMPLE_PROB {
                    regen_mh(&cube_rgbd_model, t, &orbit_azimuth_mask)
                } else {
                    mh(&cube_rgbd_model, t, &rgbd_drift, (vec!["orbit_azimuth"], 0.1))
                }
            })
            .then(|t| {
                let mut rng = ThreadRng::default();
                if u01(&mut rng) < ORBIT_RESAMPLE_PROB {
                    regen_mh(&cube_rgbd_model, t, &orbit_radius_elevation_mask)
                } else {
                    mh(&cube_rgbd_model, t, &rgbd_drift, (vec!["orbit_radius", "orbit_sin_elevation"], 0.05))
                }
            })
            .regen_mh(&lookat_jitter_mask)
            .regen_mh(&ground_albedo_mask)
            .take(SWEEPS_PER_FRAME)
            .last()
            .unwrap();

        let hyp = result.retv.clone().unwrap();
        trace = Some(result);
        hyp
    })
}
