use pose_net::{dataset, model};

use anyhow::Result;
use candle_core::Device;
use candle_nn::{loss::mse, AdamW, Optimizer, ParamsAdamW, VarMap};
use clap::Parser;
use dataset::Dataset;
use model::PoseNet;

#[derive(Parser)]
struct Args {
    #[arg(long, default_value = "out/dataset")]
    data: String,
    #[arg(long, default_value = "out/pose_net.safetensors")]
    save: String,
    #[arg(long, default_value_t = 50)]
    epochs: usize,
    #[arg(long, default_value_t = 64)]
    batch: usize,
    #[arg(long, default_value_t = 1e-3)]
    lr: f64,
    /// cap the number of samples loaded (RAM control / partial data.bin salvage)
    #[arg(long)]
    take: Option<usize>,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let device = Device::new_cuda(0)?;
    println!("training on {device:?}");

    let dataset = Dataset::load(&args.data, &Device::Cpu, args.take)?;
    println!(
        "loaded {} samples at {}x{}",
        dataset.n, dataset.h, dataset.w
    );

    let varmap = VarMap::new();
    let vb = candle_nn::VarBuilder::from_varmap(&varmap, candle_core::DType::F32, &device);
    let model = PoseNet::new(vb, dataset.h, dataset.w)?;

    let mut opt = AdamW::new(
        varmap.all_vars(),
        ParamsAdamW {
            lr: args.lr,
            ..Default::default()
        },
    )?;

    let n = dataset.n;
    let batch = args.batch;
    let mut best_loss = f64::INFINITY;
    let run_start = std::time::Instant::now();

    for epoch in 0..args.epochs {
        let epoch_start = std::time::Instant::now();
        // per-phase accumulators. Caveat: CUDA ops are queued asynchronously
        // and only forced at the to_scalar() sync, so fwd/bwd attribution is
        // approximate -- "sync" absorbs whatever was still in flight. "data"
        // (CPU gather + H2D upload) is accurate.
        let mut t_data = std::time::Duration::ZERO;
        let mut t_fwd = std::time::Duration::ZERO;
        let mut t_bwd = std::time::Duration::ZERO;
        let mut t_sync = std::time::Duration::ZERO;
        // cosine decay from lr to lr/100: a flat rate that finds the minimum
        // fast is too hot to *stay* in it -- the tail end of training needs
        // small steps or the training loss starts climbing again.
        let progress = epoch as f64 / args.epochs.max(1) as f64;
        let lr = args.lr * (0.01 + 0.99 * 0.5 * (1.0 + (std::f64::consts::PI * progress).cos()));
        opt.set_learning_rate(lr);

        let mut indices: Vec<usize> = (0..n).collect();
        fastrand_shuffle(&mut indices);

        let mut az_loss_sum = 0f64;
        let mut er_loss_sum = 0f64;
        let mut n_batches = 0usize;

        for chunk in indices.chunks(batch) {
            let t0 = std::time::Instant::now();
            let (imgs, lbls) = dataset.batch(chunk, &device)?;
            let az_target = lbls.narrow(1, 0, 2)?; // sin_az, cos_az
            let er_target = lbls.narrow(1, 2, 2)?; // sin_elev, radius
            let t1 = std::time::Instant::now();

            let (az_pred, er_pred) = model.forward(&imgs)?;
            let az_loss = mse(&az_pred, &az_target)?;
            let er_loss = mse(&er_pred, &er_target)?;
            let loss = (&az_loss + &er_loss)?;
            let t2 = std::time::Instant::now();

            opt.backward_step(&loss)?;
            let t3 = std::time::Instant::now();

            az_loss_sum += az_loss.to_scalar::<f32>()? as f64;
            er_loss_sum += er_loss.to_scalar::<f32>()? as f64;
            let t4 = std::time::Instant::now();

            t_data += t1 - t0;
            t_fwd += t2 - t1;
            t_bwd += t3 - t2;
            t_sync += t4 - t3;
            n_batches += 1;
        }

        // baselines (predict-the-mean): azimuth ~0.5, elev/radius ~0.05
        let az_loss = az_loss_sum / n_batches as f64;
        let er_loss = er_loss_sum / n_batches as f64;
        let total = az_loss + er_loss;
        let is_best = total < best_loss;

        let epoch_s = epoch_start.elapsed().as_secs_f32();
        let eta_s = run_start.elapsed().as_secs_f32() / (epoch + 1) as f32
            * (args.epochs - epoch - 1) as f32;
        println!(
            "epoch {}/{} — azimuth {az_loss:.5}  elev/radius {er_loss:.5}  lr {lr:.1e}  \
             | {epoch_s:.1}s (data {:.1} fwd {:.1} bwd {:.1} sync {:.1})  ETA {:.0}m{:02.0}s{}",
            epoch + 1,
            args.epochs,
            t_data.as_secs_f32(),
            t_fwd.as_secs_f32(),
            t_bwd.as_secs_f32(),
            t_sync.as_secs_f32(),
            (eta_s / 60.0).floor(),
            eta_s % 60.0,
            if is_best { "  *" } else { "" },
        );
        if is_best {
            best_loss = total;
            varmap.save(&args.save)?;
        }
    }

    println!("saved best (total loss {best_loss:.5}) → {}", args.save);
    Ok(())
}

fn fastrand_shuffle(v: &mut Vec<usize>) {
    use std::collections::hash_map::DefaultHasher;
    use std::hash::{Hash, Hasher};
    use std::time::SystemTime;
    let seed = SystemTime::now()
        .duration_since(SystemTime::UNIX_EPOCH)
        .unwrap()
        .as_nanos() as u64;
    let mut s = seed;
    for i in (1..v.len()).rev() {
        let mut h = DefaultHasher::new();
        s.hash(&mut h);
        s = h.finish();
        let j = (s as usize) % (i + 1);
        v.swap(i, j);
    }
}
