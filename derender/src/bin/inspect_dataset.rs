//! Dataset spot-checker: extracts samples from a gen_dataset dump and writes
//! them out as BMPs (depth + color) with their labels printed, so a human can
//! verify the renders and poses actually line up before training on them.
//!
//! Usage:
//!   cargo run --release --bin inspect_dataset -- [--data out/dataset] [--n 8]
//!
//! Picks --n samples evenly spaced through the dataset and writes
//! {data}/preview/sample_{i}_depth.bmp and _color.bmp.

use modppl_derender::serialization::{save_colors, save_depths};
use std::fs::{create_dir_all, read_to_string, File};
use std::io::{Read, Seek, SeekFrom};

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let mut data_dir = "out/dataset".to_string();
    let mut n_preview = 8usize;
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--data" => { data_dir = args.get(i+1).cloned().unwrap_or(data_dir); i += 2; }
            "--n"    => { n_preview = args.get(i+1).and_then(|s| s.parse().ok()).unwrap_or(n_preview); i += 2; }
            _ => { i += 1; }
        }
    }

    // meta first: the save helpers render at config's W()/H(), so RES must be
    // set to the dataset's resolution before anything touches the OnceLock.
    let meta = read_to_string(format!("{data_dir}/meta.txt")).expect("missing meta.txt");
    let (mut n, mut h) = (0usize, 64usize);
    for line in meta.lines() {
        if let Some(v) = line.strip_prefix("n=")      { n = v.parse().unwrap(); }
        if let Some(v) = line.strip_prefix("height=") { h = v.parse().unwrap(); }
    }
    std::env::set_var("RES", h.to_string());
    let area = h * h;
    println!("dataset: n={n}  res={h}x{h}");
    for line in meta.lines() { println!("  meta: {line}"); }

    let labels = read_to_string(format!("{data_dir}/labels.csv")).expect("missing labels.csv");
    let label_lines: Vec<&str> = labels.lines().collect(); // [0] is the header

    let preview_dir = format!("{data_dir}/preview");
    create_dir_all(&preview_dir).expect("failed to create preview dir");

    let mut f = File::open(format!("{data_dir}/data.bin")).expect("missing data.bin");
    let sample_bytes = 4 * area * 4; // 4 channels of f32

    // trust the file, not the meta: an interrupted generation leaves a short
    // data.bin behind, and previewing (or training on!) phantom samples just
    // panics halfway through.
    let actual_n = f.metadata().unwrap().len() as usize / sample_bytes;
    if actual_n < n {
        eprintln!("WARNING: data.bin holds only {actual_n}/{n} samples -- generation \
                   was interrupted. Regenerate before training.");
        n = actual_n;
    }

    println!("\n{:>6}  {}", "sample", label_lines[0]);
    for k in 0..n_preview {
        let idx = k * n / n_preview.max(1);

        f.seek(SeekFrom::Start((idx * sample_bytes) as u64)).unwrap();
        let mut buf = vec![0u8; sample_bytes];
        f.read_exact(&mut buf).expect("data.bin truncated");
        let floats: Vec<f32> = buf.chunks_exact(4)
            .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
            .collect();

        // layout per sample: depth, B, G, R -- each a contiguous H*W plane
        let depths: Vec<f32> = floats[0..area].to_vec();
        let colors: Vec<[f32; 3]> = (0..area)
            .map(|p| [floats[area + p], floats[2*area + p], floats[3*area + p]])
            .collect();

        save_depths(&format!("{preview_dir}/sample_{idx}_depth.bmp"), &depths);
        save_colors(&format!("{preview_dir}/sample_{idx}_color.bmp"), &colors);
        println!("{idx:>6}  {}", label_lines.get(idx + 1).unwrap_or(&"<missing label>"));
    }

    println!("\nwrote {n_preview} sample pairs to {preview_dir}/");
    println!("check: cube visible and multiple face colors distinguishable? \
              radius small => cube big? sin_elevation high => looking down at white face?");
}
