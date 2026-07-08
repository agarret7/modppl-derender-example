//! Windowed inference harness, independent of any model or inference
//! procedure. The tutorial sandbox: participants write a model and a `InferenceKernel`,
//! wrap them in a `restart` closure, and get an interactive convergence window
//! for free.
//!
//! Division of labor:
//! - the harness owns the window, pause/restart/snapshot/quit keys, and panel
//!   rendering (including depth auto-contrast);
//! - the `restart` closure owns everything statistical: sampling a ground
//!   truth, conditioning, building the kernel. It returns a *stepper* that
//!   runs one MCMC sweep per display frame and says what to show.
//!
//! Keys owned by the harness: Space = pause, R = restart (rebuild the stepper,
//! i.e. fresh ground truth and chain), S = snapshot to out/, ESC = quit. All
//! other pressed keys are forwarded to the stepper, so experiment-specific
//! toggles live in user code, not here.

use anyhow::Result;
use minifb::{Key, KeyRepeat, Scale, Window, WindowOptions};
use std::fs::create_dir_all;
use std::time::{SystemTime, UNIX_EPOCH};

use crate::config::{H, W};
use crate::image::{Color, Colors, Depths};
use crate::serialization::save_snapshot;

/// Window magnification, toggled via the `WINDOW_SCALE` environment variable
/// (one of 1, 2, 4, 8, 16, 32), e.g. `WINDOW_SCALE=2 cargo run --release
/// --example sandbox -- ball` for a smaller window on a small screen.
/// Defaults to 4. Unrecognized values fall back to the default.
pub fn window_scale() -> Scale {
    match std::env::var("WINDOW_SCALE").ok().as_deref() {
        Some("1") => Scale::X1,
        Some("2") => Scale::X2,
        Some("8") => Scale::X8,
        Some("16") => Scale::X16,
        Some("32") => Scale::X32,
        _ => Scale::X4,
    }
}

/// What a stepper wants on screen this frame: observation vs. hypothesis, in
/// whichever observation space the model works in.
pub enum Panels {
    /// obs | hyp, grayscale, auto-contrast-stretched to the observation's range
    Depth { obs: Depths, hyp: Depths },
    /// obs | hyp, BGR color
    Color { obs: Colors, hyp: Colors },
    /// 2x2 grid: obs depth | obs color // hyp depth | hyp color
    Rgbd {
        obs: (Depths, Colors),
        hyp: (Depths, Colors),
    },
}

/// `Color` is [f32;3] in BGR order (see image.rs); minifb wants 0x00RRGGBB.
pub(crate) fn color_to_u32(c: Color) -> u32 {
    let r = (c[2].clamp(0.0, 1.0) * 255.0) as u32;
    let g = (c[1].clamp(0.0, 1.0) * 255.0) as u32;
    let b = (c[0].clamp(0.0, 1.0) * 255.0) as u32;
    (r << 16) | (g << 8) | b
}

pub(crate) fn depth_to_u32(v: f32, lo: f32, hi: f32) -> u32 {
    if v.is_nan() {
        return 0;
    }
    let t = ((v - lo) / (hi - lo)).clamp(0.0, 1.0);
    let g = (t * 255.0) as u32;
    (g << 16) | (g << 8) | g
}

/// (min, max) over the valid (non-NaN) values, for contrast-stretching the
/// *display* independently of the likelihood's NEAR()/FAR() window.
pub(crate) fn depth_display_range(d: &[f32]) -> (f32, f32) {
    let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
    for &v in d {
        if !v.is_nan() {
            lo = lo.min(v);
            hi = hi.max(v);
        }
    }
    if !lo.is_finite() || !hi.is_finite() || hi <= lo {
        (0.0, 1.0)
    } else {
        (lo, hi)
    }
}

impl Panels {
    /// window dimensions in pixels
    pub fn dims(&self) -> (usize, usize) {
        match self {
            Panels::Rgbd { .. } => (2 * W(), 2 * H()),
            _ => (2 * W(), H()),
        }
    }

    /// fills `buf` (sized per `dims`) with the panel layout
    pub fn render(&self, buf: &mut [u32]) {
        match self {
            Panels::Depth { obs, hyp } => {
                let (lo, hi) = depth_display_range(obs);
                let stride = 2 * W();
                for y in 0..H() {
                    for x in 0..W() {
                        buf[y * stride + x] = depth_to_u32(obs[y * W() + x], lo, hi);
                        buf[y * stride + W() + x] = depth_to_u32(hyp[y * W() + x], lo, hi);
                    }
                }
            }
            Panels::Color { obs, hyp } => {
                let stride = 2 * W();
                for y in 0..H() {
                    for x in 0..W() {
                        buf[y * stride + x] = color_to_u32(obs[y * W() + x]);
                        buf[y * stride + W() + x] = color_to_u32(hyp[y * W() + x]);
                    }
                }
            }
            Panels::Rgbd {
                obs: (od, oc),
                hyp: (hd, hc),
            } => {
                let (lo, hi) = depth_display_range(od);
                let stride = 2 * W();
                for y in 0..H() {
                    for x in 0..W() {
                        buf[y * stride + x] = depth_to_u32(od[y * W() + x], lo, hi);
                        buf[y * stride + W() + x] = color_to_u32(oc[y * W() + x]);
                        buf[(H() + y) * stride + x] = depth_to_u32(hd[y * W() + x], lo, hi);
                        buf[(H() + y) * stride + W() + x] = color_to_u32(hc[y * W() + x]);
                    }
                }
            }
        }
    }
}

/// Runs the interactive loop: one stepper call (= one MCMC sweep) per display
/// frame. `restart` is invoked at startup and on every R press.
pub fn run_sandbox_loop<S>(title: &str, mut restart: impl FnMut() -> S) -> Result<()>
where
    S: FnMut(&[Key]) -> Panels,
{
    let mut stepper = restart();
    let mut panels = stepper(&[]);
    let (win_w, win_h) = panels.dims();

    let mut window = Window::new(
        title,
        win_w,
        win_h,
        WindowOptions {
            scale: window_scale(),
            ..WindowOptions::default()
        },
    )?;
    let mut buf = vec![0u32; win_w * win_h];
    let mut paused = false;

    loop {
        if !paused {
            panels.render(&mut buf);
        }
        window.update_with_buffer(&buf, win_w, win_h)?;

        if !window.is_open() || window.is_key_down(Key::Escape) {
            return Ok(());
        }

        let pressed = window.get_keys_pressed(KeyRepeat::No);
        let mut forwarded = Vec::new();
        for key in pressed {
            match key {
                Key::Space => paused = !paused,
                Key::R => {
                    stepper = restart();
                    paused = false;
                }
                Key::S => {
                    create_dir_all("out").expect("error creating 'out' dir");
                    let ts = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .unwrap()
                        .as_secs();
                    let path = format!("out/snapshot_{ts}.bmp");
                    save_snapshot(&path, &buf, win_w, win_h);
                    println!("saved {path}");
                }
                other => forwarded.push(other),
            }
        }

        if !paused {
            panels = stepper(&forwarded);
        }
    }
}
