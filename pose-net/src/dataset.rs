use anyhow::{Context, Result};
use candle_core::{Device, Tensor};
use std::fs;
use std::io::Read;

pub struct Dataset {
    /// (N, 4, H, W) f32
    pub images: Tensor,
    /// (N, 4) f32: sin_az, cos_az, sin_elev, radius
    /// (cube_u/v dropped: the orbital camera centers the cube, so position
    /// never changes the image)
    pub labels: Tensor,
    pub n: usize,
    pub h: usize,
    pub w: usize,
}

impl Dataset {
    /// `take`: optionally cap the number of samples loaded (RAM control, or
    /// salvaging a partially-generated data.bin).
    pub fn load(dir: &str, device: &Device, take: Option<usize>) -> Result<Self> {
        // read meta
        let meta = fs::read_to_string(format!("{dir}/meta.txt")).context("missing meta.txt")?;
        let mut n = 0usize;
        let mut h = 64usize;
        let mut w = 64usize;
        for line in meta.lines() {
            if let Some(v) = line.strip_prefix("n=") {
                n = v.parse()?;
            }
            if let Some(v) = line.strip_prefix("height=") {
                h = v.parse()?;
            }
            if let Some(v) = line.strip_prefix("width=") {
                w = v.parse()?;
            }
        }
        let channels = 4usize;

        // images: stream in chunks -- materializing the raw bytes and the
        // parsed floats at once doubles peak RAM, which at tens of GB of
        // dataset is the difference between loading and getting OOM-killed.
        let mut f = fs::File::open(format!("{dir}/data.bin")).context("missing data.bin")?;
        let sample_bytes = channels * h * w * 4;
        let actual_n = f.metadata()?.len() as usize / sample_bytes;
        if actual_n < n {
            eprintln!("WARNING: data.bin holds only {actual_n}/{n} samples (interrupted generation?) -- using {actual_n}");
            n = actual_n;
        }
        if let Some(t) = take {
            n = n.min(t);
        }
        let n_floats = n * channels * h * w;
        let mut floats: Vec<f32> = Vec::with_capacity(n_floats);
        let mut chunk = vec![0u8; 64 * 1024 * 1024];
        let mut remaining = n_floats * 4;
        while remaining > 0 {
            let take = chunk.len().min(remaining);
            f.read_exact(&mut chunk[..take])
                .context("data.bin truncated")?;
            floats.extend(
                chunk[..take]
                    .chunks_exact(4)
                    .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])),
            );
            remaining -= take;
        }
        let images = Tensor::from_vec(floats, (n, channels, h, w), device)?;

        // labels: CSV columns: azimuth, sin_elevation, radius, cube_u, cube_v
        // we encode azimuth as (sin, cos) → 6 output columns total
        let csv = fs::read_to_string(format!("{dir}/labels.csv")).context("missing labels.csv")?;
        let mut label_vec: Vec<f32> = Vec::with_capacity(n * 6);
        for line in csv.lines().skip(1).take(n) {
            let cols: Vec<f64> = line
                .split(',')
                .map(|s| s.trim().parse::<f64>().unwrap_or(0.0))
                .collect();
            if cols.len() < 5 {
                continue;
            }
            let (az, sin_elev, radius, u, v) = (cols[0], cols[1], cols[2], cols[3], cols[4]);
            // azimuth dropped: a symmetric cube looks identical from any azimuth,
            // so it's unlearnable and only adds noise to the loss
            let _ = (u, v);
            label_vec.push(az.sin() as f32);
            label_vec.push(az.cos() as f32);
            label_vec.push(sin_elev as f32);
            label_vec.push(radius as f32);
        }
        let labels = Tensor::from_vec(label_vec, (n, 4), device)?;

        Ok(Self {
            images,
            labels,
            n,
            h,
            w,
        })
    }

    /// Returns (images, labels) minibatch tensors: a single index_select
    /// gather on the CPU tensor, then one upload -- per-sample narrow+cat
    /// was the training loop's bottleneck at dataset sizes in the GBs.
    pub fn batch(&self, indices: &[usize], device: &Device) -> Result<(Tensor, Tensor)> {
        let idx = Tensor::from_vec(
            indices.iter().map(|&i| i as u32).collect::<Vec<u32>>(),
            indices.len(),
            self.images.device(),
        )?;
        Ok((
            self.images.index_select(&idx, 0)?.to_device(device)?,
            self.labels.index_select(&idx, 0)?.to_device(device)?,
        ))
    }
}
