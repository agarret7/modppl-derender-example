use modppl::prelude::*;
use modppl_derender::{
    core::*,
    config::{W, H},
    image::Color,
    inference::Kernel,
    realsense::*,
    serialization::save_snapshot,
};
use minifb::{Window, WindowOptions, Key, KeyRepeat, Scale};
use std::fs::create_dir_all;
use std::time::{SystemTime, UNIX_EPOCH};

// bumped up from (0.1, 0.2) to better reflect a noisier/older depth camera:
// more dropout/jitter in depth, more lighting/white-balance drift in color,
// than an idealized sensor. Too-confident a likelihood treats real sensor
// noise as signal, driving spurious inference moves instead of absorbing it.
const DEPTH_NOISE: f32 = 0.2;
const COLOR_NOISE: f32 = 0.32;
const SWEEPS_PER_FRAME: usize = 5;

/// finds the (min, max) of the valid (non-NaN/dropout) values in a depth
/// buffer, for auto-contrast-stretching the *display* -- independent of the
/// likelihood's NEAR()/FAR() window, which has to stay wide enough to tolerate
/// the camera moving around the orbit. Without this, the full NEAR()..FAR()
/// window (much wider than a close-up scene's actual depth variation) gets
/// linearly mapped to grayscale, so real depth gets squashed into a narrow,
/// washed-out band of gray.
fn depth_display_range(d: &[f32]) -> (f32, f32) {
    let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
    for &v in d {
        if v.is_nan() { continue; }
        lo = lo.min(v);
        hi = hi.max(v);
    }
    if !lo.is_finite() || !hi.is_finite() || hi <= lo { (0.0, 1.0) } else { (lo, hi) }
}

/// `lm` is the model's normalized inverse-depth convention: 1.0 at NEAR, 0.0 at/beyond FAR.
/// `(lo, hi)` is the display's contrast-stretch range (see `depth_display_range`).
fn lm_to_u32(v: f32, lo: f32, hi: f32) -> u32 {
    if v.is_nan() { return 0; }
    let t = ((v - lo) / (hi - lo)).clamp(0.0, 1.0);
    let g = (t * 255.0) as u32;
    (g << 16) | (g << 8) | g
}

/// `Color` is [f32;3] in BGR order (see image.rs); minifb wants 0x00RRGGBB.
fn color_to_u32(c: Color) -> u32 {
    let r = (c[2].clamp(0.0, 1.0) * 255.0) as u32;
    let g = (c[1].clamp(0.0, 1.0) * 255.0) as u32;
    let b = (c[0].clamp(0.0, 1.0) * 255.0) as u32;
    (r << 16) | (g << 8) | b
}

fn main() -> anyhow::Result<()> {
    let (mut pipeline, mut align) = open_rgbd_pipeline()?;
    println!("RealSense RGB-D pipeline open, capturing first frame...");

    // initialize the trace from the first observed frame
    let (depth_frame, color_frame) = read_rgbd_frames(&mut pipeline, &mut align)?;

    // calibrate the renderer's FOV from the real camera's intrinsics *before*
    // any model call (config::FOVY() is fixed on first read) -- a guessed FOV
    // would make the renderer disagree with the real camera on how large a
    // given physical object should appear at a given distance.
    let fovy = calibrate_fovy(&color_frame)?;
    std::env::set_var("FOVY_RAD", fovy.to_string());
    println!("calibrated vertical FOV: {:.1} deg", fovy.to_degrees());

    // sanity-check units empirically rather than trusting the docs: print the
    // raw center-pixel distance so it can be compared against a tape measure.
    // DepthFrame::distance() is documented to already return metric meters
    // (depth_units applied internally), but that's the *original* depth
    // stream's behavior -- worth confirming it still holds after `Align`
    // remaps the frame onto the color stream's pixel grid.
    if let Ok(center_m) = depth_frame.distance(CAM_W/2, CAM_H/2) {
        println!("center-pixel raw distance: {center_m:.3} m  (hold the cube here and compare to a tape measure)");
    }

    let (observed_depth, observed_color) = rgbd_from_frames(&depth_frame, &color_frame);
    let mut constraints = DynTrie::new();
    constraints.observe("depth_observation", Arc::new(observed_depth));
    constraints.observe("color_observation", Arc::new(observed_color));
    let mut trace = cube_rgbd_model.generate((DEPTH_NOISE, COLOR_NOISE), constraints).0;

    let mut cube_mask = AddrMap::new();
    cube_mask.visit("cube_u");
    cube_mask.visit("cube_v");

    let mut orbit_azimuth_mask = AddrMap::new();
    orbit_azimuth_mask.visit("orbit_azimuth");

    let mut orbit_radius_elevation_mask = AddrMap::new();
    orbit_radius_elevation_mask.visit("orbit_radius");
    orbit_radius_elevation_mask.visit("orbit_sin_elevation");

    // tight prior (normal(0, 0.05) rad) already keeps proposals small, so a
    // plain regen_mh resample is fine here (no jump risk like orbit_angle's
    // full 0..2pi range).
    let mut lookat_jitter_mask = AddrMap::new();
    lookat_jitter_mask.visit("lookat_yaw_jitter");
    lookat_jitter_mask.visit("lookat_pitch_jitter");

    // strongly identified by every ground-hit pixel's color (unlike the
    // weakly-constrained orbit latents), so a plain regen_mh resample each
    // sweep is fine -- no drift/mixture treatment needed.
    let mut ground_albedo_mask = AddrMap::new();
    ground_albedo_mask.visit("ground_albedo");

    // 2x2 grid: observed depth | observed color  //  hypothesis depth | hypothesis color
    let mut window = Window::new(
        "live RGB-D cube derender  |  observed : hypothesis  |  S to snapshot, ESC to quit",
        2*W(), 2*H(),
        WindowOptions { scale: Scale::X4, ..WindowOptions::default() }
    )?;

    let mut buffer = vec![0u32; 2*W()*2*H()];

    // probability of a full uniform/regen_mh resample instead of a drift step,
    // for the orbit latents -- mostly drift (smooth tracking), occasionally a
    // wide move so the chain can recover if tracking is lost or started wrong.
    const ORBIT_RESAMPLE_PROB: f64 = 0.1;

    while window.is_open() && !window.is_key_down(Key::Escape) {
        // capture a new frame and swap it in, warm-starting from the current trace
        let (depth_frame, color_frame) = read_rgbd_frames(&mut pipeline, &mut align)?;
        let (observed_depth, observed_color) = rgbd_from_frames(&depth_frame, &color_frame);

        let mut constraints = DynTrie::new();
        constraints.observe("depth_observation", Arc::new(observed_depth.clone()));
        constraints.observe("color_observation", Arc::new(observed_color.clone()));
        let (new_trace, _discard, _weight) = cube_rgbd_model.update(
            trace, (DEPTH_NOISE, COLOR_NOISE), ArgDiff::NoChange, constraints
        );
        trace = new_trace;

        // a few MH sweeps to adapt the latents to the new observation.
        // orbit pose is a mixture move: mostly drift (small steps from its
        // current value, for smooth tracking), with an occasional full
        // regen_mh resample from the prior (for recovery) -- pure drift can't
        // escape a bad lock, but a full resample every frame would let the
        // camera jump to point anywhere on the orbit each frame, since a
        // single small object barely constrains where on the orbit it is.
        trace = Kernel::new(&cube_rgbd_model, trace)
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

        let (hyp_depth, hyp_color) = trace.retv.clone().unwrap();

        // stretch both depth panels to the *observed* frame's actual range, so
        // they stay on the same visual scale and aren't washed out by the
        // likelihood's much wider NEAR()..FAR() window.
        let (depth_lo, depth_hi) = depth_display_range(&observed_depth);

        let stride = 2*W();
        for y in 0..H() {
            for x in 0..W() {
                buffer[y*stride + x]         = lm_to_u32(observed_depth[y*W() + x], depth_lo, depth_hi);
                buffer[y*stride + W() + x]   = color_to_u32(observed_color[y*W() + x]);
                buffer[(H()+y)*stride + x]       = lm_to_u32(hyp_depth[y*W() + x], depth_lo, depth_hi);
                buffer[(H()+y)*stride + W() + x] = color_to_u32(hyp_color[y*W() + x]);
            }
        }

        window.update_with_buffer(&buffer, 2*W(), 2*H())?;

        // S: dump the current 2x2 grid (observed | hypothesis) to out/ on demand
        if window.is_key_pressed(Key::S, KeyRepeat::No) {
            create_dir_all("out").expect("error creating 'out' dir");
            let ts = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
            let path = format!("out/snapshot_{ts}.bmp");
            save_snapshot(&path, &buffer, 2*W(), 2*H());
            println!("saved {path}");
        }
    }

    Ok(())
}
