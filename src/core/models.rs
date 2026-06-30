use std::f32::consts::PI;
use float_extras::f64::erf;
use glam::{Mat4, Affine3A, Vec3, Vec3A, Quat, EulerRot};
use modppl::prelude::*;

// use crate::baseline::linear::*;
// use crate::baseline::solid::*;
// use crate::baseline::ray::*;
use crate::core::solid::*;
use crate::core::ray::*;
use crate::config::*;
use crate::image::*;


/* pixel likelihoods */

/// truncated Gaussian distribution type
pub struct TruncatedNormal { }
pub const truncated_normal: TruncatedNormal = TruncatedNormal { };

fn normal_cdf(x: &f64, params: (f64,f64)) -> f64 {
    let (mu, sigma) = params;
    let xi = (x - mu) / sigma;
    0.5*(1. + erf(xi as f64/2f64.sqrt()))
}

impl Distribution<f32,(f32,f32,f32,f32)> for TruncatedNormal {
    fn logpdf(&self, x: &f32, params: (f32,f32,f32,f32)) -> f64 {
        let (mu, sigma, a, b) = params;
        if a <= *x && *x <= b {
            normal.logpdf(&(*x as f64), (mu as f64, sigma as f64)) 
              - (normal_cdf(&(b as f64), (mu as f64, sigma as f64)) -
                 normal_cdf(&(a as f64), (mu as f64, sigma as f64))).ln()
              - ((sigma as f64).ln())
        } else {
            f64::NEG_INFINITY
        }
    }

    fn random(&self, rng: &mut ThreadRng, params: (f32,f32,f32,f32)) -> f32 {
        let (mu, sigma, a, b) = params;
        let mut x = normal.random(rng, (mu as f64, sigma as f64)) as f32;
        while !(a <= x && x <= b) {  // rejection sampling
            x = normal.random(rng, (mu as f64, sigma as f64)) as f32;
        }
        x
    }
}

/// noisy depth distribution type
struct NoisyDepths { }
const noisy_depths: NoisyDepths = NoisyDepths { };

impl Distribution<Depths,(Depths,f32)> for NoisyDepths {
    fn logpdf(&self, noisy_pixels: &Depths, pixels_and_noise: (Depths,f32)) -> f64 {
        let (pixels, noise) = pixels_and_noise;
        let mut w = 0.;
        for y in 0..H() {
            for x in 0..W() {
                let noisy_p = noisy_pixels[y*W() + x];
                // NaN marks a real sensor dropout (no measurement, see
                // realsense::depths_from_frame) -- not a real value, so it
                // carries no evidence and must be excluded, not scored as if
                // it meant "background."  Synthetic renders never produce NaN.
                if noisy_p.is_nan() { continue; }
                let true_p = pixels[y*W() + x];
                w += truncated_normal.logpdf(&noisy_p, (true_p, noise, 0.0, 1.0))
            }
        }
        (1. - noise as f64)*(w as f64)
    }

    fn random(&self, rng: &mut ThreadRng, pixels_and_noise: (Depths,f32)) -> Depths {
        let (pixels, noise) = pixels_and_noise;
        let noise_f64 = noise as f64;
        let mut noisy_pixels = vec![];
        for y in 0..H() {
            for x in 0..W() {
                // Add mixture of noise from uniform and gaussian
                if u01(rng) < noise_f64 {
                    noisy_pixels.push(u01(rng) as f32);
                } else {
                    let noisy_p = truncated_normal.random(rng, (pixels[y*W() + x], noise, 0.0, 1.0));
                    noisy_pixels.push(noisy_p as f32);
                }
            }
        }
        noisy_pixels
    }
}

/// noisy (isotropic) color distribution type
struct NoisyColors { }
const noisy_colors: NoisyColors = NoisyColors { };

impl Distribution<Colors,(Colors,f32)> for NoisyColors {
    fn logpdf(&self, noisy_pixels: &Colors, pixels_and_noise: (Colors,f32)) -> f64 {
        let (pixels, noise) = pixels_and_noise;
        let mut w = 0.;
        for y in 0..H() {
            for x in 0..W() {
                for i in 0..=2 {
                    let noisy_p = noisy_pixels[y*W() + x][i];
                    let true_p = pixels[y*W() + x][i];
                    w += truncated_normal.logpdf(&noisy_p, (true_p, noise, 0.0, 1.0))
                }
            }
        }
        (1. - noise as f64)*(w as f64)
    }

    fn random(&self, rng: &mut ThreadRng, pixels_and_noise: (Colors,f32)) -> Colors {
        let (pixels, noise) = pixels_and_noise;
        let noise_f64 = noise as f64;
        let mut noisy_pixels = vec![];
        for y in 0..H() {
            for x in 0..W() {
                // Add mixture of noise from uniform and gaussian
                let mut noisy_p = [0.0; 3];
                for i in 0..=2 {
                    if u01(rng) < noise_f64 {
                        noisy_p[i] = u01(rng) as f32;
                    } else {
                        noisy_p[i] = truncated_normal.random(rng, (pixels[y*W() + x][i], noise, 0., 1.)) as f32;
                    }
                }
                noisy_pixels.push(noisy_p)
            }
        }
        noisy_pixels
    }
}


/* dynamic generative functions */

dyngen!(
pub fn grounded_depth_model() -> Depths {
    // camera pose
    let cam_y = uniform(0.5, 2.0) %= "cam_y";
    let cam_roll = normal(0.0, PI as f64/8.0) %= "cam_roll";
    let x = Affine3A::from_rotation_translation(
        Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, cam_roll as f32),
        [0.0, cam_y as f32, 1.2].into()
    );

    // ground
    let ground = (
        Box::new(Plane { origin: Vec3A::ZERO, normal: [0.0, 1.0, 0.0].into() }) as Box<dyn Solid>,
        [0.0, 0.0, 0.0]
    );

    // render
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32/H() as f32, NEAR(), FAR());
    let mut pixels = vec![0.0; AREA()];
    raytrace_depths(x, proj, &vec![ground], &mut pixels);
    noisy_depths(pixels.clone(), 0.1) %= "observation";

    pixels
});

dyngen!(
pub fn sphere_color_model() -> Colors {
    // camera pose
    let cam_y = uniform(0.5, 2.0) %= "cam_y";
    let cam_roll = normal(0.0, PI as f64/8.0) %= "cam_roll";
    let x = Affine3A::from_rotation_translation(
        Quat::from_euler(EulerRot::XYZ, 0.0, 0.0, cam_roll as f32),
        [0.0, cam_y as f32, 1.2].into()
    );

    // background
    let brightness = (uniform(0.5, 1.0) %= "ambient_brightness") as f32;
    let background_color = [brightness, brightness, brightness];

    // ground
    let ground_albedo = (uniform(0.0, 1.0) %= "ground_albedo") as f32;
    let ground = (
        Box::new(Plane { origin: Vec3A::ZERO, normal: [0.0, 1.0, 0.0].into() }) as Box<dyn Solid>,
        [ground_albedo, ground_albedo, ground_albedo]
    );

    // sphere
    let u = (uniform(-2.0, 2.0) %= "sphere_u") as f32;
    let v = (uniform(-2.0, 0.0) %= "sphere_v") as f32;
    let redness = (uniform(0.0, 1.0) %= "sphere_redness") as f32;
    let sphere = (
        Box::new(Sphere { center: [u, 0.5, v].into(), radius: 0.5 }) as Box<dyn Solid>,
        [0.2, 1.0 - redness, redness]
    );

    // render
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32/H() as f32, NEAR(), FAR());
    let mut pixels = vec![[0.0; 3]; AREA()];
    raytrace_colors(x, proj, &vec![ground, sphere], background_color, &mut pixels);
    noisy_colors(pixels.clone(), 0.1) %= "observation";

    pixels
});

dyngen!(
pub fn ball_model() -> Colors {
    // camera pose
    let cam_y = uniform(0.5, 2.0) %= "cam_y";
    let cam_yaw = normal(0.0, PI as f64/8.0) %= "cam_yaw";
    let x = Affine3A::from_rotation_translation(
        Quat::from_euler(EulerRot::XYZ, cam_yaw as f32, 0.0, 0.0),
        [0.0, cam_y as f32, 1.2].into()
    );

    // background
    let b = (uniform(0.75, 1.0) %= "ambient_brightness") as f32;
    let background_c = [0.9 * b, 1.0 * b, 1.0 * b];

    // ground
    let mut table_c = [0.0; 3];
    table_c[0] = (uniform(0.0, 1.0) %= "table_c0") as f32;
    table_c[1] = (uniform(0.0, 1.0) %= "table_c1") as f32;
    table_c[2] = (uniform(0.0, 1.0) %= "table_c2") as f32;
    let table = (
        Box::new(Plane { origin: Vec3A::ZERO, normal: [0.0, 1.0, 0.0].into() }) as Box<dyn Solid>,
        table_c
    );

    // ball
    let u = (uniform(-1.0, 1.0) %= "ball_u") as f32;
    let v = (uniform(-1.0, 0.0) %= "ball_v") as f32;
    let mut ball_c = [0.0; 3];
    ball_c[0] = (uniform(0.25, 1.0) %= "ball_c0") as f32;
    ball_c[1] = (uniform(0.25, 1.0) %= "ball_c1") as f32;
    ball_c[2] = (uniform(0.25, 1.0) %= "ball_c2") as f32;
    let ball_r = (uniform(0.3, 0.5) %= "ball_radius") as f32;
    let ball = (
        Box::new(Sphere { center: [u, ball_r, v].into(), radius: ball_r }) as Box<dyn Solid>,
        ball_c
    );

    // render
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32/H() as f32, NEAR(), FAR());
    let mut pixels = vec![[0.0; 3]; AREA()];
    raytrace_colors(x, proj, &vec![table, ball], background_c, &mut pixels);
    noisy_colors(pixels.clone(), 0.1) %= "observation";

    pixels
});

dyngen!(
pub fn mug_model(noise: f32) -> Colors {
    // camera pose
    let cam_y = uniform(0.5, 2.0) %= "cam_y";
    let cam_yaw = normal(0.0, PI as f64/8.0) %= "cam_yaw";
    let x = Affine3A::from_rotation_translation(
        Quat::from_euler(EulerRot::XYZ, cam_yaw as f32, 0.0, 0.0),
        [0.0, cam_y as f32, 1.2].into()
    );

    // background
    let b = (uniform(0.75, 1.0) %= "ambient_brightness") as f32;
    let background_c = [0.9 * b, 1.0 * b, 1.0 * b];

    // table
    let mut table_c = [0.0; 3];
    table_c[0] = (uniform(0.0, 1.0) %= "table_c0") as f32;
    table_c[1] = (uniform(0.0, 1.0) %= "table_c1") as f32;
    table_c[2] = (uniform(0.0, 1.0) %= "table_c2") as f32;
    let table = (
        Box::new(Plane { origin: Vec3A::ZERO, normal: [0.0, 1.0, 0.0].into() }) as Box<dyn Solid>,
        table_c
    );

    // mug: a cylinder standing on the table, with unknown color and pose
    let u = (uniform(-1.0, 1.0) %= "mug_u") as f32;
    let v = (uniform(-1.0, 0.0) %= "mug_v") as f32;
    let mug_radius = (uniform(0.2, 0.4) %= "mug_radius") as f32;
    let mug_height = (uniform(0.4, 0.8) %= "mug_height") as f32;
    let mut mug_c = [0.0; 3];
    mug_c[0] = (uniform(0.25, 1.0) %= "mug_c0") as f32;
    mug_c[1] = (uniform(0.25, 1.0) %= "mug_c1") as f32;
    mug_c[2] = (uniform(0.25, 1.0) %= "mug_c2") as f32;
    let mug = (
        Box::new(Cylinder { base: [u, 0.0, v].into(), radius: mug_radius, height: mug_height }) as Box<dyn Solid>,
        mug_c
    );

    // render
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32/H() as f32, NEAR(), FAR());
    let mut pixels = vec![[0.0; 3]; AREA()];
    raytrace_colors(x, proj, &vec![table, mug], background_c, &mut pixels);
    noisy_colors(pixels.clone(), noise) %= "observation";

    pixels
});

dyngen!(
pub fn rubiks_model(noise: f32) -> Colors {
    // camera pose
    let cam_y = uniform(0.5, 2.0) %= "cam_y";
    let cam_yaw = normal(0.0, PI as f64/8.0) %= "cam_yaw";
    let x = Affine3A::from_rotation_translation(
        Quat::from_euler(EulerRot::XYZ, cam_yaw as f32, 0.0, 0.0),
        [0.0, cam_y as f32, 1.2].into()
    );

    // background
    let b = (uniform(0.75, 1.0) %= "ambient_brightness") as f32;
    let background_c = [0.9 * b, 1.0 * b, 1.0 * b];

    // table
    let mut table_c = [0.0; 3];
    table_c[0] = (uniform(0.0, 1.0) %= "table_c0") as f32;
    table_c[1] = (uniform(0.0, 1.0) %= "table_c1") as f32;
    table_c[2] = (uniform(0.0, 1.0) %= "table_c2") as f32;
    let table = (
        Box::new(Plane { origin: Vec3A::ZERO, normal: [0.0, 1.0, 0.0].into() }) as Box<dyn Solid>,
        table_c
    );

    // cube: a Rubik's cube resting on the table, with unknown position and size.
    // Color is fixed per-face by `Cube::color_at`, so it carries no unknown color
    // latents (unlike `mug_model`'s cylinder).
    let u = (uniform(-1.0, 1.0) %= "cube_u") as f32;
    let v = (uniform(-1.0, 0.0) %= "cube_v") as f32;
    let half_extent = (uniform(0.2, 0.4) %= "cube_size") as f32;
    let cube = (
        Box::new(Cube { center: [u, half_extent, v].into(), half_extent }) as Box<dyn Solid>,
        [0.0, 0.0, 0.0] // ignored: Cube::color_at overrides per-face
    );

    // render
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32/H() as f32, NEAR(), FAR());
    let mut pixels = vec![[0.0; 3]; AREA()];
    raytrace_colors(x, proj, &vec![table, cube], background_c, &mut pixels);
    noisy_colors(pixels.clone(), noise) %= "observation";

    pixels
});

dyngen!(
/// depth-only cube model, for derendering a real RealSense depth stream: a cube
/// resting on the ground plane with unknown position/size, scored against a
/// depth observation rather than color (no lighting/material sim-to-real gap).
pub fn cube_depth_model(noise: f32) -> Depths {
    // camera pose
    let cam_y = uniform(0.5, 2.0) %= "cam_y";
    let cam_yaw = normal(0.0, PI as f64/8.0) %= "cam_yaw";
    let x = Affine3A::from_rotation_translation(
        Quat::from_euler(EulerRot::XYZ, cam_yaw as f32, 0.0, 0.0),
        [0.0, cam_y as f32, 1.2].into()
    );

    // ground
    let ground = (
        Box::new(Plane { origin: Vec3A::ZERO, normal: [0.0, 1.0, 0.0].into() }) as Box<dyn Solid>,
        [0.0, 0.0, 0.0]
    );

    // cube: unknown position and size, resting on the ground
    let u = (uniform(-1.0, 1.0) %= "cube_u") as f32;
    let v = (uniform(-1.0, 0.0) %= "cube_v") as f32;
    let half_extent = (uniform(0.05, 0.5) %= "cube_size") as f32;
    let cube = (
        Box::new(Cube { center: [u, half_extent, v].into(), half_extent }) as Box<dyn Solid>,
        [0.0, 0.0, 0.0] // unused: depth rendering ignores color
    );

    // render
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32/H() as f32, NEAR(), FAR());
    let mut pixels = vec![0.0; AREA()];
    raytrace_depths(x, proj, &vec![ground, cube], &mut pixels);
    noisy_depths(pixels.clone(), noise) %= "observation";

    pixels
});

dyngen!(
pub fn depth_drift(trace: Weak<DynTrace<f32,Depths>>, mask: Vec<&str>, stdev: f64) {
    let trace = trace.upgrade().unwrap();
    for addr in mask.iter() {
        normal(trace.data.read::<f64>(addr), stdev) %= addr;
    }
});

/// builds a camera-to-world transform for a camera on a hemisphere around
/// `target` (the camera can't orbit below the table) at the given `azimuth`
/// (radians, full circle) and `cos_elevation` (elevation measured from the
/// horizon, 0 = level with `target`, pi/2 = directly overhead -- the max
/// angle, since the hemisphere's pole is straight up), at distance `radius`,
/// looking at `target` plus a small angular offset (`jitter_yaw`,
/// `jitter_pitch`, in radians).
///
/// Takes `cos(elevation)` rather than the elevation angle itself: for a
/// *uniform* prior over the hemisphere's surface/solid angle, the elevation
/// can't be sampled uniformly -- the hemisphere's area element is `sin(theta)
/// dtheta dphi`, so equal angular steps near the pole sweep much less area
/// than near the horizon, biasing toward overhead positions. Sampling
/// `cos(elevation) ~ Uniform(0,1)` (and azimuth ~ Uniform(0,2pi)) corrects
/// for this and gives true uniform-area coverage.
///
/// The look-at is offset by a small jitter rather than pinned exactly to
/// `target`: an exact look-at couples camera orientation tightly to the
/// cube's hypothesized position, so a small position proposal also swings the
/// camera to re-center on it, shifting the *entire* rendered frame (not just
/// the cube) -- amplifying a small positional change into a large change in
/// the rendered image. The jitter decouples the two, restoring a smooth,
/// local relationship between small position changes and small image changes.
fn orbit_camera(azimuth: f32, cos_elevation: f32, radius: f32, target: Vec3A, jitter_yaw: f32, jitter_pitch: f32) -> Affine3A {
    let sin_elevation = (1.0 - cos_elevation*cos_elevation).sqrt();
    let height = radius * sin_elevation;
    let horiz = radius * cos_elevation;
    let eye: Vec3 = (target + Vec3A::new(horiz * azimuth.cos(), height, horiz * azimuth.sin())).into();
    let view = Affine3A::look_at_rh(eye, target.into(), Vec3::Y);
    let jitter = Affine3A::from_quat(Quat::from_euler(EulerRot::YXZ, jitter_yaw, jitter_pitch, 0.0));
    view.inverse() * jitter
}

dyngen!(
/// depth + flat-color cube model, for derendering a real RealSense RGB-D stream
/// of a physically multi-colored cube (e.g. an actual Rubik's cube): depth
/// handles position/size/pose (no lighting/material sim-to-real gap), while a
/// *flat*, no-lighting color render (`raytrace_flat_colors`) handles which face
/// is which (orientation) -- viable because the stickers are deliberately flat
/// and highly saturated, so skipping lighting simulation entirely is *more*
/// faithful to how they actually look than path-traced shading would be.
///
/// The camera is modeled as orbiting the cube (not an independent height/yaw),
/// with a look-at constraint toward the cube's center: a depth observation of a
/// single small object barely constrains where the camera is, so an
/// unconstrained pose latent tends to wander to wherever still explains the
/// data, including poses not actually pointed at the cube. Orbiting + look-at
/// guarantees the cube stays in frame regardless of the (still uncertain)
/// orbit position.
pub fn cube_rgbd_model(noise: (f32,f32)) -> (Depths, Colors) {
    let (depth_noise, color_noise) = noise;

    // cube: unknown position, resting on the ground. Size is a known constant
    // (a real Rubik's cube is ~4.5cm wide), not inferred -- with a known size,
    // orbit_radius is the only free scale parameter, so depth alone identifies
    // it cleanly. Inferring both size and distance from a single small object
    // is a classic scale/depth ambiguity (see cube_depth_model/cube_size above
    // for the latent version).
    const CUBE_HALF_EXTENT: f32 = 0.0225;

    let u = (uniform(-1.0, 1.0) %= "cube_u") as f32;
    let v = (uniform(-1.0, 0.0) %= "cube_v") as f32;
    let cube = (
        Box::new(Cube { center: [u, CUBE_HALF_EXTENT, v].into(), half_extent: CUBE_HALF_EXTENT }) as Box<dyn Solid>,
        [0.0, 0.0, 0.0] // unused: Cube::color_at overrides per-face
    );

    // camera: uniform over the hemisphere above the cube (see orbit_camera's
    // doc comment for why cos_elevation, not elevation, must be the latent),
    // looking at it plus a small angular jitter. radius range (0.25-0.7m)
    // brackets an actual handheld operating distance of ~0.3-0.6m. jitter
    // stdev (~3 deg) decouples camera orientation from the cube's exact
    // hypothesized position.
    let orbit_azimuth = (uniform(0.0, 2.0*PI as f64) %= "orbit_azimuth") as f32;
    let orbit_cos_elevation = (uniform(0.0, 1.0) %= "orbit_cos_elevation") as f32;
    let orbit_radius = (uniform(0.25, 0.7) %= "orbit_radius") as f32;
    let lookat_yaw_jitter = (normal(0.0, 0.05) %= "lookat_yaw_jitter") as f32;
    let lookat_pitch_jitter = (normal(0.0, 0.05) %= "lookat_pitch_jitter") as f32;
    let x = orbit_camera(
        orbit_azimuth, orbit_cos_elevation, orbit_radius, [u, CUBE_HALF_EXTENT, v].into(),
        lookat_yaw_jitter, lookat_pitch_jitter
    );

    // ground
    let ground = (
        Box::new(Plane { origin: Vec3A::ZERO, normal: [0.0, 1.0, 0.0].into() }) as Box<dyn Solid>,
        [0.5, 0.5, 0.5]
    );

    let scene = vec![ground, cube];
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32/H() as f32, NEAR(), FAR());

    // depth observation: position/size/pose
    let mut depths = vec![0.0; AREA()];
    raytrace_depths(x, proj, &scene, &mut depths);
    noisy_depths(depths.clone(), depth_noise) %= "depth_observation";

    // flat color observation: which face is which
    let mut colors = vec![[0.0; 3]; AREA()];
    raytrace_flat_colors(x, proj, &scene, [0.7, 0.7, 0.7], &mut colors);
    noisy_colors(colors.clone(), color_noise) %= "color_observation";

    (depths, colors)
});

dyngen!(
pub fn rgbd_drift(trace: Weak<DynTrace<(f32,f32),(Depths,Colors)>>, mask: Vec<&str>, stdev: f64) {
    let trace = trace.upgrade().unwrap();
    for addr in mask.iter() {
        normal(trace.data.read::<f64>(addr), stdev) %= addr;
    }
});

dyngen!(
pub fn gaussian_drift(trace: Weak<DynTrace<(),Colors>>, mask: Vec<&str>, stdev: f64) {
    let trace = trace.upgrade().unwrap();
    for addr in mask.iter() {
        normal(trace.data.read::<f64>(addr), stdev) %= addr;
    }
});

dyngen!(
pub fn noise_drift(trace: Weak<DynTrace<f32,Colors>>, mask: Vec<&str>, stdev: f64) {
    let trace = trace.upgrade().unwrap();
    for addr in mask.iter() {
        normal(trace.data.read::<f64>(addr), stdev) %= addr;
    }
});