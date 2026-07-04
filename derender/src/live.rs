//! Real-time RGB-D inference loop, decoupled from any particular model.
//!
//! Call [`run_rgbd_loop`] with a `step` closure that owns all model and
//! inference state. The loop handles camera setup, FOV calibration, display,
//! and key bindings; the closure handles everything inference-specific.

use anyhow::Result;
use minifb::{Key, KeyRepeat, Scale, Window, WindowOptions};
use std::fs::create_dir_all;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::config::{H, W};
use crate::image::{Colors, Depths};
use crate::realsense::*;
use crate::sandbox::Panels;
use crate::serialization::save_snapshot;

// ── main loop ────────────────────────────────────────────────────────────────

/// Runs a real-time RGB-D derender loop against a RealSense D400-series camera.
///
/// `step(obs_depth, obs_color) -> (hyp_depth, hyp_color)` is called once per
/// camera frame. It should warm-start the current trace against the new
/// observation, run a fixed number of MCMC sweeps, and return the hypothesis
/// render. All model and inference state lives inside the closure.
///
/// The loop handles everything else: camera setup, FOV calibration (sets
/// `FOVY_RAD` before the first `step` call), a 2×2 minifb window
/// (observed depth | observed color / hypothesis depth | hypothesis color),
/// auto-contrast depth display, S to snapshot, ESC to quit.
pub fn run_rgbd_loop(mut step: impl FnMut(Depths, Colors) -> (Depths, Colors)) -> Result<()> {
    let (mut pipeline, mut align) = open_rgbd_pipeline()?;
    println!("RealSense RGB-D pipeline open, capturing first frame...");

    // read the first frame before anything else so we can calibrate FOV
    let (depth_frame, color_frame) = read_rgbd_frames(&mut pipeline, &mut align)?;

    // set FOVY_RAD before any model call so the OnceLock picks up the real value
    let fovy = calibrate_fovy(&color_frame)?;
    std::env::set_var("FOVY_RAD", fovy.to_string());
    println!("calibrated vertical FOV: {:.1} deg", fovy.to_degrees());

    if let Ok(m) = depth_frame.distance(CAM_W / 2, CAM_H / 2) {
        println!("center-pixel distance: {m:.3} m");
    }

    let mut window = Window::new(
        "live RGB-D derender  |  observed (top) : hypothesis (bottom)  |  S=snapshot  ESC=quit",
        2 * W(), 2 * H(),
        WindowOptions { scale: Scale::X4, ..WindowOptions::default() },
    )?;
    let mut buffer = vec![0u32; 2 * W() * 2 * H()];

    // bootstrap: process first frame before entering the main loop
    let mut obs = rgbd_from_frames(&depth_frame, &color_frame);

    loop {
        let (obs_depth, obs_color) = obs;
        let (hyp_depth, hyp_color) = step(obs_depth.clone(), obs_color.clone());

        Panels::Rgbd { obs: (obs_depth, obs_color), hyp: (hyp_depth, hyp_color) }
            .render(&mut buffer);

        window.update_with_buffer(&buffer, 2 * W(), 2 * H())?;

        if window.is_key_pressed(Key::S, KeyRepeat::No) {
            create_dir_all("out").expect("error creating 'out' dir");
            let ts = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
            let path = format!("out/snapshot_{ts}.bmp");
            save_snapshot(&path, &buffer, 2 * W(), 2 * H());
            println!("saved {path}");
        }

        if !window.is_open() || window.is_key_down(Key::Escape) { break; }

        let (df, cf) = read_rgbd_frames(&mut pipeline, &mut align)?;
        obs = rgbd_from_frames(&df, &cf);
    }

    Ok(())
}
