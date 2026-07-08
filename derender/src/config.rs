use std::env;
use std::f32::consts::PI;
use std::sync::OnceLock;

/* constants */

static RESOLUTION: OnceLock<usize> = OnceLock::new();
static NEAR_PLANE: OnceLock<f32> = OnceLock::new();
static FAR_PLANE: OnceLock<f32> = OnceLock::new();
static FOVY_RAD: OnceLock<f32> = OnceLock::new();

/// Render resolution (width == height), toggled via the `RES` environment
/// variable, e.g. `RES=128 cargo test --release --test derender test_derender_mug`.
/// Defaults to 64. Fixed for the lifetime of the process once first read.
fn resolution() -> usize {
    *RESOLUTION.get_or_init(|| {
        env::var("RES")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(64)
    })
}

pub fn H() -> usize {
    resolution()
}
pub fn W() -> usize {
    resolution()
}
pub fn AREA() -> usize {
    H() * W()
}
pub fn VP() -> [f32; 4] {
    [0.0, 0.0, W() as f32, H() as f32]
}

/// Vertical field of view, in radians. Defaults to PI/2 (matches the synthetic
/// tutorial models' assumed camera). For the live RealSense demo, a mismatched
/// FOV is a real bug, not cosmetic: the renderer assumes whatever angle is set
/// here, so a wider-than-real FOV spreads the same physical object over more of
/// the frame, making it render smaller than the real camera shows it (and vice
/// versa). Override via `FOVY_RAD` -- set from the camera's *actual* measured
/// intrinsics (see `realsense::calibrate_fovy`), not guessed.
pub fn FOVY() -> f32 {
    *FOVY_RAD.get_or_init(|| {
        env::var("FOVY_RAD")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(PI / 2.0)
    })
}

/// Near/far clipping planes, in scene units (meters for the live RealSense demo).
/// Depth is normalized against this range (1.0 at NEAR, 0.0 at/beyond FAR), so a
/// range much wider than the actual working volume wastes most of the [0,1]
/// dynamic range and makes close-up scenes look flat. Toggle via `NEAR_M`/`FAR_M`
/// env vars, e.g. `NEAR_M=0.15 FAR_M=1.0 cargo run --release --bin live_cube`
/// for a close-up tabletop scene. Defaults (0.2, 7.5) suit the synthetic tutorial
/// models' world scale and are unaffected unless overridden.
pub fn NEAR() -> f32 {
    *NEAR_PLANE.get_or_init(|| {
        env::var("NEAR_M")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(0.2)
    })
}
pub fn FAR() -> f32 {
    *FAR_PLANE.get_or_init(|| {
        env::var("FAR_M")
            .ok()
            .and_then(|s| s.parse().ok())
            .unwrap_or(7.5)
    })
}

/// Sets the canonical `cube_rgbd_model` pipeline environment (RES=128,
/// FOVY_RAD=0.74 ≈ the D435 color stream's vertical FOV, NEAR_M=0.01,
/// FAR_M=1.0) as *defaults*: explicitly-set env vars still win. Call this at
/// the top of any binary that only ever uses the cube model (dataset gen,
/// CNN train/validate, live demos), before anything reads the config -- the
/// CNN bakes FOV and the depth encoding into its weights, so these values
/// must agree across dataset generation, training, and inference, and
/// requiring four env vars on every command is how mismatches happen.
pub fn apply_cube_pipeline_defaults() {
    for (key, val) in [
        ("RES", "128"),
        ("FOVY_RAD", "0.74"),
        ("NEAR_M", "0.01"),
        ("FAR_M", "1.0"),
    ] {
        if env::var(key).is_err() {
            env::set_var(key, val);
        }
    }
}
