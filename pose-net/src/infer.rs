use pose_net::model;

use anyhow::Result;
use candle_core::{Device, Tensor, DType};
use candle_nn::VarBuilder;
use clap::Parser;
use model::PoseNet;

#[derive(Parser)]
struct Args {
    /// path to saved weights (.safetensors)
    #[arg(long, default_value = "out/pose_net.safetensors")]
    weights: String,
    /// raw (4, H, W) f32 image bytes on stdin, or path to a single sample's image slice
    #[arg(long)]
    image: String,
    #[arg(long, default_value_t = 64)]
    h: usize,
    #[arg(long, default_value_t = 64)]
    w: usize,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let device = Device::new_cuda(0).unwrap_or(Device::Cpu);

    let vb = unsafe {
        VarBuilder::from_mmaped_safetensors(&[&args.weights], DType::F32, &device)?
    };
    let model = PoseNet::new(vb, args.h, args.w)?;

    // load a single (4, H, W) image from a raw binary slice of data.bin
    let bytes = std::fs::read(&args.image)?;
    let floats: Vec<f32> = bytes.chunks_exact(4)
        .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]]))
        .collect();
    let img = Tensor::from_vec(floats, (1, 4, args.h, args.w), &device)?;

    let (az_pred, er_pred) = model.forward(&img)?;
    let az = az_pred.squeeze(0)?.to_vec1::<f32>()?;
    let er = er_pred.squeeze(0)?.to_vec1::<f32>()?;

    let azimuth = az[0].atan2(az[1]);
    let confidence = (az[0] * az[0] + az[1] * az[1]).sqrt();
    println!("azimuth:       {azimuth:.3} rad  (confidence {confidence:.2})");
    println!("sin_elevation: {:.3}", er[0]);
    println!("radius:        {:.3} m", er[1]);

    Ok(())
}
