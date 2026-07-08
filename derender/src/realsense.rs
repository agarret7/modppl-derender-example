use anyhow::{ensure, Result};
use realsense_rust::{
    config::Config,
    context::Context,
    frame::{ColorFrame, DepthFrame, FrameEx, PixelKind},
    kind::{Rs2CameraInfo, Rs2Format, Rs2ProductLine, Rs2StreamKind},
    pipeline::{ActivePipeline, InactivePipeline},
    processing_blocks::align::Align,
};
use std::{collections::HashSet, time::Duration};

use crate::config::{FAR, H, NEAR, W};
use crate::image::{Colors, Depths};

/// native D435 depth stream resolution; we downsample (with aspect "squash",
/// not crop) to the model's square `W()`x`H()` render resolution.
pub const CAM_W: usize = 640;
pub const CAM_H: usize = 480;

pub fn open_depth_pipeline() -> Result<ActivePipeline> {
    let mut queried_devices = HashSet::new();
    queried_devices.insert(Rs2ProductLine::D400);
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    ensure!(!devices.is_empty(), "No RealSense device found");

    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();
    config
        .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
        .disable_all_streams()?
        .enable_stream(Rs2StreamKind::Depth, None, CAM_W, CAM_H, Rs2Format::Z16, 30)?;

    Ok(pipeline.start(Some(config))?)
}

pub fn read_depth_frame(pipeline: &mut ActivePipeline) -> Result<DepthFrame> {
    let timeout = Duration::from_millis(5000);
    let frames = pipeline.wait(Some(timeout))?;
    Ok(frames
        .frames_of_type::<DepthFrame>()
        .pop()
        .expect("no depth frame in frameset"))
}

/// converts a raw depth frame into the model's normalized `Depths` representation
/// (matching `raytrace_depths`'s convention: 1.0 at `NEAR()`, 0.0 at/beyond
/// `FAR()`), downsampling from `CAM_W`x`CAM_H` to `W()`x`H()`. Sensor dropout
/// (`distance() <= 0`, librealsense2's invalid-pixel sentinel) is encoded as
/// `NaN`, distinct from a legitimate out-of-range reading -- both used to
/// collapse to the same `0.0` as a real depth-miss, but a dropout is "we don't
/// know," not "it's far away," and scoring it as background actively
/// misinforms the likelihood. `NoisyDepths::logpdf` skips `NaN` pixels entirely.
pub fn depths_from_frame(frame: &DepthFrame) -> Depths {
    let mut out = vec![0.0; W() * H()];
    for y in 0..H() {
        for x in 0..W() {
            let cam_x = x * CAM_W / W();
            let cam_y = y * CAM_H / H();
            let d = frame.distance(cam_x, cam_y).unwrap_or(0.0);
            out[y * W() + x] = if d <= 0.0 {
                f32::NAN
            } else if d <= FAR() {
                // saturates at 1.0 below NEAR, matching the renderer's clamp
                (1.0 - (d - NEAR()) / (FAR() - NEAR())).clamp(0.0, 1.0)
            } else {
                0.0
            };
        }
    }
    out
}

/// opens depth + color streams together, aligned to the color frame's pixel grid
/// (depth and color have different native fields of view, so without alignment
/// the two wouldn't correspond pixel-for-pixel).
pub fn open_rgbd_pipeline() -> Result<(ActivePipeline, Align)> {
    let mut queried_devices = HashSet::new();
    queried_devices.insert(Rs2ProductLine::D400);
    let context = Context::new()?;
    let devices = context.query_devices(queried_devices);
    ensure!(!devices.is_empty(), "No RealSense device found");

    let pipeline = InactivePipeline::try_from(&context)?;
    let mut config = Config::new();
    config
        .enable_device_from_serial(devices[0].info(Rs2CameraInfo::SerialNumber).unwrap())?
        .disable_all_streams()?
        .enable_stream(Rs2StreamKind::Depth, None, CAM_W, CAM_H, Rs2Format::Z16, 30)?
        .enable_stream(
            Rs2StreamKind::Color,
            None,
            CAM_W,
            CAM_H,
            Rs2Format::Rgb8,
            30,
        )?;

    let pipeline = pipeline.start(Some(config))?;
    let align = Align::new(Rs2StreamKind::Color, 8)?;
    Ok((pipeline, align))
}

pub fn read_rgbd_frames(
    pipeline: &mut ActivePipeline,
    align: &mut Align,
) -> Result<(DepthFrame, ColorFrame)> {
    let timeout = Duration::from_millis(5000);
    let frames = pipeline.wait(Some(timeout))?;
    align.queue(frames)?;
    let aligned = align.wait(timeout)?;
    let depth = aligned
        .frames_of_type::<DepthFrame>()
        .pop()
        .expect("no depth frame in aligned frameset");
    let color = aligned
        .frames_of_type::<ColorFrame>()
        .pop()
        .expect("no color frame in aligned frameset");
    Ok((depth, color))
}

/// converts aligned depth+color frames into the model's `(Depths, Colors)`
/// representation. `Colors` is `[f32;3]` in BGR order (see image.rs), matching
/// the BMP-native layout the rest of the renderer assumes.
pub fn rgbd_from_frames(depth: &DepthFrame, color: &ColorFrame) -> (Depths, Colors) {
    let mut depths = vec![0.0; W() * H()];
    let mut colors = vec![[0.0; 3]; W() * H()];
    for y in 0..H() {
        for x in 0..W() {
            let cam_x = x * CAM_W / W();
            let cam_y = y * CAM_H / H();

            let d = depth.distance(cam_x, cam_y).unwrap_or(0.0);
            depths[y * W() + x] = if d <= 0.0 {
                f32::NAN
            } else if d <= FAR() {
                // saturates at 1.0 below NEAR, matching the renderer's clamp
                (1.0 - (d - NEAR()) / (FAR() - NEAR())).clamp(0.0, 1.0)
            } else {
                0.0
            };

            if let Some(PixelKind::Bgr8 { r, g, b }) = color.get(cam_x, cam_y) {
                colors[y * W() + x] = [*b as f32 / 255.0, *g as f32 / 255.0, *r as f32 / 255.0];
            }
        }
    }
    (depths, colors)
}

/// computes the camera's real vertical field of view (in radians) from a color
/// frame's actual intrinsics, rather than guessing -- a wrong FOV is a real bug:
/// it makes the renderer's perspective projection disagree with the real
/// camera's, so objects at the correct inferred distance still render at the
/// wrong pixel size. Set the result via `std::env::set_var("FOVY_RAD", ...)`
/// *before* the first call to `config::FOVY()` (it's a OnceLock, fixed on first read).
pub fn calibrate_fovy(color: &ColorFrame) -> Result<f32> {
    let intrinsics = color.stream_profile().intrinsics()?;
    Ok(2.0 * (intrinsics.height() as f32 / 2.0 / intrinsics.fy()).atan())
}
