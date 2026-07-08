use candle_core::{Result, Tensor};
use candle_nn::{conv2d, linear, Conv2d, Conv2dConfig, Linear, Module, VarBuilder};

use crate::arch::{conv_arch, N_AZ_OUTPUTS, N_ER_OUTPUTS};

/// Small CNN: (B, 4, H, W) → pose estimate, two heads.
/// Input channels: depth, B, G, R.
///
/// Stride-2 convs instead of maxpool: azimuth is read from the *arrangement*
/// of the cube's face colors (red left-of blue, etc.), and per-channel max
/// over a 2x2 window throws away exactly that within-window position
/// information. A strided conv downsamples with learned weights instead.
/// Five stride-2 layers also grow the receptive field to cover the whole
/// image -- the previous 3-conv/pool stack topped out at ~15 px, less than
/// one close-up cube (~26 px at 128 res), so no conv feature could ever see
/// the full face arrangement at once.
///
/// Head split rationale: elevation/radius are readable from the whole frame
/// (the ground plane's depth gradient), while azimuth lives only in the
/// cube's few face pixels -- separate heads keep the easy targets from
/// hiding whether the hard one learns.
pub struct PoseNet {
    convs: Vec<Conv2d>,
    fc: Linear,
    head_az: Linear, // (sin_az, cos_az)
    head_er: Linear, // (sin_elevation, radius)
}

impl PoseNet {
    /// `h`/`w`: input image size. 64 and 128 square are supported: 128 uses
    /// all five stride-2 layers (→ 4x4), 64 skips the last downsample by
    /// running the final conv at stride 1 (8x8 → less reduction but same
    /// channel count, so only the fc input size differs).
    pub fn new(vb: VarBuilder, h: usize, w: usize) -> Result<Self> {
        let (layers, final_cells) =
            conv_arch(h, w).map_err(|err| candle_core::Error::Msg(err.to_string()))?;

        let mut convs = Vec::new();
        for layer in layers {
            let cfg = Conv2dConfig {
                padding: 1,
                stride: layer.stride,
                ..Default::default()
            };
            convs.push(conv2d(
                layer.cin,
                layer.cout,
                3,
                cfg,
                vb.pp(format!("conv{}", layer.index)),
            )?);
        }

        Ok(Self {
            convs,
            fc: linear(256 * final_cells, 256, vb.pp("fc"))?,
            head_az: linear(256, N_AZ_OUTPUTS, vb.pp("head_az"))?,
            head_er: linear(256, N_ER_OUTPUTS, vb.pp("head_er"))?,
        })
    }

    /// Returns (azimuth_head, elev_radius_head): (B,2) each.
    /// Azimuth head is (sin, cos) -- decode with atan2; its norm doubles as a
    /// confidence signal (MSE shrinks it toward zero under ambiguous views,
    /// e.g. the 4-fold symmetry looking straight down at the white face).
    pub fn forward(&self, x: &Tensor) -> Result<(Tensor, Tensor)> {
        let mut x = x.clone();
        for conv in &self.convs {
            x = conv.forward(&x)?.relu()?;
        }
        let (b, c, hh, ww) = x.dims4()?;
        let x = x.reshape((b, c * hh * ww))?;
        let x = self.fc.forward(&x)?.relu()?;
        Ok((self.head_az.forward(&x)?, self.head_er.forward(&x)?))
    }
}
