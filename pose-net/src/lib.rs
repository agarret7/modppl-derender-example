pub mod arch;
pub mod dataset;
pub mod model;

use anyhow::Result;
use candle_core::{DType, Device, Tensor};
use candle_nn::VarBuilder;
use model::PoseNet;

pub struct PoseEstimate {
    /// [0, 2π), matching the model's orbit_azimuth convention
    pub azimuth: f32,
    /// norm of the (sin, cos) head output: ~1 on confident views, shrinks
    /// toward 0 on ambiguous ones (e.g. looking straight down at the white
    /// face) -- MSE training hedges multimodal views toward the circle
    /// center, so the norm is a free ambiguity signal.
    pub azimuth_confidence: f32,
    pub sin_elevation: f32,
    pub radius: f32,
}

/// Loads a trained PoseNet and runs single-frame pose estimation, in the
/// derender project's image conventions.
pub struct PoseEstimator {
    model: PoseNet,
    device: Device,
    h: usize,
    w: usize,
}

impl PoseEstimator {
    /// `h`/`w` must match the resolution the weights were trained at
    /// (see the dataset's meta.txt).
    pub fn load(weights: &str, h: usize, w: usize) -> Result<Self> {
        let device = Device::new_cuda(0).unwrap_or(Device::Cpu);
        let vb = unsafe { VarBuilder::from_mmaped_safetensors(&[weights], DType::F32, &device)? };
        Ok(Self {
            model: PoseNet::new(vb, h, w)?,
            device,
            h,
            w,
        })
    }

    /// `depths`: H*W normalized inverse-depth; NaN dropout is allowed and
    /// mapped to 0.0 ("at/beyond far"), which is in-distribution for the
    /// synthetic training renders -- raw NaN would poison every conv output.
    /// `colors`: H*W BGR in 0-1.
    pub fn estimate(&self, depths: &[f32], colors: &[[f32; 3]]) -> Result<PoseEstimate> {
        let area = self.h * self.w;
        anyhow::ensure!(
            depths.len() == area && colors.len() == area,
            "resolution mismatch: estimator expects {}x{} (got {} depth px) -- \
             run with RES matching the training resolution",
            self.h,
            self.w,
            depths.len()
        );

        // (1, 4, H, W): depth plane then B, G, R planes, same as gen_dataset
        let mut buf = Vec::with_capacity(4 * area);
        buf.extend(depths.iter().map(|d| if d.is_nan() { 0.0 } else { *d }));
        for ch in 0..3 {
            buf.extend(colors.iter().map(|c| c[ch]));
        }
        let img = Tensor::from_vec(buf, (1, 4, self.h, self.w), &self.device)?;

        let (az, er) = self.model.forward(&img)?;
        let az = az.squeeze(0)?.to_vec1::<f32>()?;
        let er = er.squeeze(0)?.to_vec1::<f32>()?;

        Ok(PoseEstimate {
            azimuth: az[0].atan2(az[1]).rem_euclid(std::f32::consts::TAU),
            azimuth_confidence: (az[0] * az[0] + az[1] * az[1]).sqrt(),
            sin_elevation: er[0],
            radius: er[1],
        })
    }
}
