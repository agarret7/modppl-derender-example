//! Live-window synthetic derender demo for the ModPPL tutorial.
//!
//! Shows a synthesized observation (left) and the inferred hypothesis (right)
//! updating in real time as MCMC runs.
//!
//! Keys:
//!   Space — pause / resume inference
//!   R     — resample a new observation and restart inference
//!   S     — save a snapshot of the current frame to out/
//!   ESC   — quit
//!
//! Usage:
//!   cargo run --release --bin derender_window [ground|sphere|mug|rubiks]
//!
//! Resolution can be tuned with the RES env var (default 64):
//!   RES=128 cargo run --release --bin derender_window sphere

use modppl::prelude::*;
use modppl_derender::{
    core::*,
    config::{W, H},
    image::{Color, Colors, Depths},
    inference::Kernel,
    serialization::save_snapshot,
};
use minifb::{Key, KeyRepeat, Scale, Window, WindowOptions};
use std::fs::create_dir_all;
use std::time::{SystemTime, UNIX_EPOCH};

// ── display helpers ──────────────────────────────────────────────────────────

fn color_to_u32(c: Color) -> u32 {
    let r = (c[2].clamp(0.0, 1.0) * 255.0) as u32;
    let g = (c[1].clamp(0.0, 1.0) * 255.0) as u32;
    let b = (c[0].clamp(0.0, 1.0) * 255.0) as u32;
    (r << 16) | (g << 8) | b
}

fn depth_to_u32(v: f32, lo: f32, hi: f32) -> u32 {
    if v.is_nan() { return 0; }
    let t = ((v - lo) / (hi - lo)).clamp(0.0, 1.0);
    let g = (t * 255.0) as u32;
    (g << 16) | (g << 8) | g
}

fn depths_to_buffer(obs: &Depths, hyp: &Depths) -> Vec<u32> {
    let (mut lo, mut hi) = (f32::INFINITY, f32::NEG_INFINITY);
    for &v in obs {
        if !v.is_nan() { lo = lo.min(v); hi = hi.max(v); }
    }
    if !lo.is_finite() || hi <= lo { lo = 0.0; hi = 1.0; }

    let mut buf = vec![0u32; 2 * W() * H()];
    for y in 0..H() {
        for x in 0..W() {
            buf[y * 2 * W() + x]       = depth_to_u32(obs[y * W() + x], lo, hi);
            buf[y * 2 * W() + W() + x] = depth_to_u32(hyp[y * W() + x], lo, hi);
        }
    }
    buf
}

fn colors_to_buffer(obs: &Colors, hyp: &Colors) -> Vec<u32> {
    let mut buf = vec![0u32; 2 * W() * H()];
    for y in 0..H() {
        for x in 0..W() {
            buf[y * 2 * W() + x]       = color_to_u32(obs[y * W() + x]);
            buf[y * 2 * W() + W() + x] = color_to_u32(hyp[y * W() + x]);
        }
    }
    buf
}

fn open_window(title: &str) -> Window {
    Window::new(
        title, 2 * W(), H(),
        WindowOptions { scale: Scale::X4, ..WindowOptions::default() },
    ).expect("failed to open minifb window")
}

/// Pushes buffer to the window. Returns false when the window should close.
fn push(window: &mut Window, buf: &[u32]) -> bool {
    window.update_with_buffer(buf, 2 * W(), H()).is_ok()
        && window.is_open()
        && !window.is_key_down(Key::Escape)
}

fn snapshot(buf: &[u32]) {
    create_dir_all("out").expect("error creating 'out' dir");
    let ts  = SystemTime::now().duration_since(UNIX_EPOCH).unwrap().as_secs();
    let path = format!("out/snapshot_{ts}.bmp");
    save_snapshot(&path, buf, 2 * W(), H());
    println!("saved {path}");
}

// ── model-specific inference loops ──────────────────────────────────────────

fn run_ground() {
    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_roll");
    cam_mask.visit("cam_y");

    let mut window = open_window(
        "derender: ground  |  obs (left)  hyp (right)  |  Space=pause  R=resample  S=save  ESC=quit"
    );
    let mut paused   = false;
    let mut last_buf = vec![0u32; 2 * W() * H()];

    'restart: loop {
        let mut synth = DynTrie::new();
        synth.observe("cam_roll", Arc::new(0.0_f64));
        synth.observe("cam_y",    Arc::new(1.5_f64));
        let gt          = grounded_depth_model.generate((), synth).0;
        let observation = gt.data.read::<Depths>("observation").clone();

        let mut constraints = DynTrie::new();
        constraints.observe("observation", Arc::new(observation.clone()));
        let trace = grounded_depth_model.generate((), constraints).0;

        let mut kernel = Kernel::new(&grounded_depth_model, trace)
            .regen_mh(&cam_mask);

        loop {
            let buf = if !paused {
                let t   = kernel.next().unwrap();
                let hyp = t.retv.as_ref().unwrap();
                let b   = depths_to_buffer(&observation, hyp);
                last_buf = b.clone();
                b
            } else {
                last_buf.clone()
            };

            if !push(&mut window, &buf) { return; }

            if window.is_key_pressed(Key::S,     KeyRepeat::No) { snapshot(&buf); }
            if window.is_key_pressed(Key::Space,  KeyRepeat::No) { paused = !paused; }
            if window.is_key_pressed(Key::R,      KeyRepeat::No) { continue 'restart; }
        }
    }
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

    let mut window = open_window(
        "derender: sphere  |  obs (left)  hyp (right)  |  Space=pause  R=resample  S=save  ESC=quit"
    );
    let mut paused   = false;
    let mut last_buf = vec![0u32; 2 * W() * H()];

    'restart: loop {
        let mut synth = DynTrie::new();
        synth.observe("cam_roll",           Arc::new(0.0_f64));
        synth.observe("ground_albedo",      Arc::new(0.5_f64));
        synth.observe("ambient_brightness", Arc::new(0.95_f64));
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

        loop {
            let buf = if !paused {
                let t   = kernel.next().unwrap();
                let hyp = t.retv.as_ref().unwrap();
                let b   = colors_to_buffer(&observation, hyp);
                last_buf = b.clone();
                b
            } else {
                last_buf.clone()
            };

            if !push(&mut window, &buf) { return; }

            if window.is_key_pressed(Key::S,     KeyRepeat::No) { snapshot(&buf); }
            if window.is_key_pressed(Key::Space,  KeyRepeat::No) { paused = !paused; }
            if window.is_key_pressed(Key::R,      KeyRepeat::No) { continue 'restart; }
        }
    }
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

    let mut window = open_window(
        "derender: mug  |  obs (left)  hyp (right)  |  Space=pause  R=resample  S=save  ESC=quit"
    );
    let mut paused   = false;
    let mut last_buf = vec![0u32; 2 * W() * H()];

    'restart: loop {
        let mut synth = DynTrie::new();
        synth.observe("cam_yaw", Arc::new(0.0_f64));
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

        loop {
            let buf = if !paused {
                let t   = kernel.next().unwrap();
                let hyp = t.retv.as_ref().unwrap();
                let b   = colors_to_buffer(&observation, hyp);
                last_buf = b.clone();
                b
            } else {
                last_buf.clone()
            };

            if !push(&mut window, &buf) { return; }

            if window.is_key_pressed(Key::S,     KeyRepeat::No) { snapshot(&buf); }
            if window.is_key_pressed(Key::Space,  KeyRepeat::No) { paused = !paused; }
            if window.is_key_pressed(Key::R,      KeyRepeat::No) { continue 'restart; }
        }
    }
}

fn run_rubiks() {
    let mut orbit_azimuth_mask = AddrMap::new();
    orbit_azimuth_mask.visit("orbit_azimuth");

    let mut orbit_elev_radius_mask = AddrMap::new();
    orbit_elev_radius_mask.visit("orbit_sin_elevation");
    orbit_elev_radius_mask.visit("orbit_radius");

    let mut lookat_jitter_mask = AddrMap::new();
    lookat_jitter_mask.visit("lookat_yaw_jitter");
    lookat_jitter_mask.visit("lookat_pitch_jitter");

    let mut env_mask = AddrMap::new();
    env_mask.visit("table_c0");
    env_mask.visit("table_c1");
    env_mask.visit("table_c2");
    env_mask.visit("ambient_brightness");

    let mut cube_mask = AddrMap::new();
    cube_mask.visit("cube_u");
    cube_mask.visit("cube_v");
    cube_mask.visit("cube_size");

    let mut window = open_window(
        "derender: rubiks  |  obs (left)  hyp (right)  |  Space=pause  R=resample  S=save  ESC=quit"
    );
    let mut paused   = false;
    let mut last_buf = vec![0u32; 2 * W() * H()];

    'restart: loop {
        // orbital camera always points at the cube, no constraints needed
        let gt          = rubiks_model.generate(0.05_f32, DynTrie::new()).0;
        let observation = gt.data.read::<Colors>("observation").clone();

        let mut constraints = DynTrie::new();
        constraints.observe("observation", Arc::new(observation.clone()));
        let trace = rubiks_model.generate(0.05_f32, constraints).0;

        let mut kernel = Kernel::new(&rubiks_model, trace)
            .regen_mh(&orbit_azimuth_mask)
            .regen_mh(&orbit_elev_radius_mask)
            .regen_mh(&lookat_jitter_mask)
            .regen_mh(&env_mask)
            .regen_mh(&cube_mask)
            .mh(&noise_drift, (vec!["cube_u", "cube_v", "cube_size"], 0.1));

        loop {
            let buf = if !paused {
                let t   = kernel.next().unwrap();
                let hyp = t.retv.as_ref().unwrap();
                let b   = colors_to_buffer(&observation, hyp);
                last_buf = b.clone();
                b
            } else {
                last_buf.clone()
            };

            if !push(&mut window, &buf) { return; }

            if window.is_key_pressed(Key::S,     KeyRepeat::No) { snapshot(&buf); }
            if window.is_key_pressed(Key::Space,  KeyRepeat::No) { paused = !paused; }
            if window.is_key_pressed(Key::R,      KeyRepeat::No) { continue 'restart; }
        }
    }
}

// ── entry point ──────────────────────────────────────────────────────────────

fn main() {
    let model = std::env::args().nth(1).unwrap_or_else(|| "ground".to_string());
    match model.as_str() {
        "ground" => run_ground(),
        "sphere" => run_sphere(),
        "mug"    => run_mug(),
        "rubiks" => run_rubiks(),
        other    => eprintln!(
            "unknown model '{other}'\nusage: derender_window [ground|sphere|mug|rubiks]"
        ),
    }
}
