//! Synthetic dataset generator for CNN pose estimation training.
//!
//! Samples from `cube_rgbd_model` and writes a dataset to disk:
//!   out/dataset/data.bin   — (N, 4, H, W) f32, channels: depth, B, G, R
//!   out/dataset/labels.csv — N rows: azimuth, sin_elevation, radius, cube_u, cube_v
//!
//! Usage:
//!   cargo run --release --bin gen_dataset -- [--n 10000] [--path-trace] [--out out/dataset]
//!
//! Defaults: 10 000 samples, flat renderer, out/dataset/.
//! Use --path-trace for more realistic color at ~10x slower generation.

use modppl::prelude::*;
use modppl_derender::{
    core::*,
    config::{W, H, AREA, FOVY, NEAR, FAR},
};
use std::fs::{create_dir_all, File};
use std::io::{BufWriter, Write};
use std::time::Instant;

fn parse_args() -> (usize, bool, String) {
    let args: Vec<String> = std::env::args().collect();
    let mut n = 10_000usize;
    let mut path_trace = false;
    let mut out = "out/dataset".to_string();
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--n" => { n = args.get(i+1).and_then(|s| s.parse().ok()).unwrap_or(n); i += 2; }
            "--path-trace" => { path_trace = true; i += 1; }
            "--out" => { out = args.get(i+1).cloned().unwrap_or(out); i += 2; }
            _ => { i += 1; }
        }
    }
    (n, path_trace, out)
}

fn main() {
    modppl_derender::config::apply_cube_pipeline_defaults();
    let (n, path_trace, out_dir) = parse_args();
    println!(
        "generating {n} samples  renderer={}  res={}x{}  fovy={:.1}deg  out={out_dir}",
        if path_trace { "path" } else { "flat" }, W(), H(), FOVY().to_degrees()
    );
    create_dir_all(&out_dir).expect("failed to create output directory");

    // (N, 4, H, W) f32 image tensor written sample-by-sample
    let data_path = format!("{out_dir}/data.bin");
    let mut data_file = BufWriter::new(File::create(&data_path).expect("failed to create data.bin"));

    // CSV: azimuth, sin_elevation, radius, cube_u, cube_v
    let labels_path = format!("{out_dir}/labels.csv");
    let mut labels_file = BufWriter::new(File::create(&labels_path).expect("failed to create labels.csv"));
    writeln!(labels_file, "azimuth,sin_elevation,radius,cube_u,cube_v").unwrap();

    // metadata so the training script knows the tensor shape without parsing everything
    let meta_path = format!("{out_dir}/meta.txt");
    // fovy/near/far are part of what the CNN learns (apparent size at a given
    // distance; the depth channel's normalization), so a training/inference
    // mismatch in any of them silently degrades the estimator. Record them.
    std::fs::write(&meta_path, format!(
        "n={n}\nchannels=4\nheight={}\nwidth={}\nfovy_rad={}\nnear_m={}\nfar_m={}\nrenderer={}\n",
        H(), W(), FOVY(), NEAR(), FAR(), if path_trace { "path" } else { "flat" }
    )).unwrap();

    let start = Instant::now();
    let report_every = (n / 20).max(1);

    for i in 0..n {
        let trace = cube_rgbd_model.generate((0.0, 0.0, path_trace), DynTrie::new()).0;

        // read latents before consuming retv
        let azimuth  = trace.data.read::<f32>("orbit_azimuth");
        let sin_elev = trace.data.read::<f32>("orbit_sin_elevation");
        let radius   = trace.data.read::<f32>("orbit_radius");
        let cube_u   = trace.data.read::<f32>("cube_u");
        let cube_v   = trace.data.read::<f32>("cube_v");

        let (depths, colors) = trace.retv.unwrap();

        // write (4, H, W) f32: depth channel then B, G, R channels
        let depth_bytes: Vec<u8> = depths.iter().flat_map(|&v| v.to_le_bytes()).collect();
        data_file.write_all(&depth_bytes).unwrap();
        for ch in 0..3 {
            let ch_bytes: Vec<u8> = colors.iter().flat_map(|c| c[ch].to_le_bytes()).collect();
            data_file.write_all(&ch_bytes).unwrap();
        }

        writeln!(labels_file, "{azimuth:.6},{sin_elev:.6},{radius:.6},{cube_u:.6},{cube_v:.6}").unwrap();

        if (i + 1) % report_every == 0 {
            let elapsed = start.elapsed().as_secs_f32();
            let rate = (i + 1) as f32 / elapsed;
            let eta = (n - i - 1) as f32 / rate;
            println!("  {}/{n}  {rate:.0} samples/s  ETA {eta:.0}s", i + 1);
        }
    }

    data_file.flush().unwrap();
    labels_file.flush().unwrap();

    let gb = (n * 4 * AREA() * 4) as f64 / 1e9;
    println!(
        "done in {:.1}s  →  {data_path} ({gb:.2} GB)  {labels_path}",
        start.elapsed().as_secs_f32()
    );
}
