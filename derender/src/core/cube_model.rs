//! The one production Rubik's-cube model: real-world metric scale, joint
//! depth+color observations. Used unchanged by `gen_dataset`, `live_rgbd_cube`,
//! pose-net's `live_cnn`/`synth_cnn`, and the `test_derender_rubiks` test --
//! genuinely shared across binaries (unlike the other, example-only scene
//! models), so it stays in the library rather than moving to `examples/scenes`.
//! `examples/scenes/rubiks.rs` keeps its own self-contained copy for editing.

use glam::{Affine3A, EulerRot, Mat4, Quat, Vec3, Vec3A};
use modppl::prelude::*;
use std::f32::consts::PI;

use crate::config::*;
use crate::core::likelihoods::{noisy_colors, noisy_depths};
use crate::core::ray::*;
use crate::core::solid::*;
use crate::image::*;

/// builds a camera-to-world transform for a camera on a hemisphere around
/// `target` at the given `azimuth` (radians, full circle) and `sin_elevation`
/// (sine of the elevation above the horizon, 0 = level, 1 = directly overhead),
/// at distance `radius`, looking at `target` plus a small angular offset
/// (`jitter_yaw`, `jitter_pitch`, in radians).
///
/// Takes `sin(elevation)` rather than the elevation angle itself: the
/// hemisphere area element in (elevation, azimuth) is `cos(elevation) de dφ`,
/// so sampling elevation uniformly biases toward the pole. Sampling
/// `sin(elevation) ~ Uniform(0,1)` gives pdf ∝ cos(elevation), which exactly
/// cancels the area element and produces uniform solid-angle coverage.
///
/// The look-at is offset by a small jitter rather than pinned exactly to
/// `target`: an exact look-at couples camera orientation tightly to the
/// cube's hypothesized position, so a small position proposal also swings the
/// camera to re-center on it, shifting the *entire* rendered frame (not just
/// the cube) -- amplifying a small positional change into a large change in
/// the rendered image. The jitter decouples the two, restoring a smooth,
/// local relationship between small position changes and small image changes.
fn orbit_camera(
    azimuth: f32,
    sin_elevation: f32,
    radius: f32,
    target: Vec3A,
    jitter_yaw: f32,
    jitter_pitch: f32,
) -> Affine3A {
    let cos_elevation = (1.0 - sin_elevation * sin_elevation).sqrt();
    let height = radius * sin_elevation;
    let horiz = radius * cos_elevation;
    let eye: Vec3 =
        (target + Vec3A::new(horiz * azimuth.cos(), height, horiz * azimuth.sin())).into();
    let view = Affine3A::look_at_rh(eye, target.into(), Vec3::Y);
    let jitter = Affine3A::from_quat(Quat::from_euler(
        EulerRot::YXZ,
        jitter_yaw,
        jitter_pitch,
        0.0,
    ));
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
    pub fn cube_rgbd_model(noise: (f32, f32, bool)) -> (Depths, Colors) {
        let (depth_noise, color_noise, path_trace) = noise;

        // cube: unknown position, resting on the ground. Size is a known constant
        // (a real Rubik's cube is ~4.5cm wide), not inferred -- with a known size,
        // orbit_radius is the only free scale parameter, so depth alone identifies
        // it cleanly. Inferring both size and distance from a single small object
        // is a classic scale/depth ambiguity.
        const CUBE_HALF_EXTENT: f32 = 0.0225;

        let u = uniform(-1.0, 1.0) %= "cube_u";
        let v = uniform(-1.0, 0.0) %= "cube_v";
        let cube = (
            Box::new(Cube {
                center: [u, CUBE_HALF_EXTENT, v].into(),
                half_extent: CUBE_HALF_EXTENT,
            }) as Box<dyn Solid>,
            [0.0, 0.0, 0.0], // unused: Cube::color_at overrides per-face
        );

        // camera: uniform over the hemisphere above the cube (see orbit_camera's
        // doc comment for why sin_elevation, not elevation, must be the latent),
        // looking at it plus a small angular jitter. radius range (0.25-0.7m)
        // brackets an actual handheld operating distance of ~0.3-0.6m. jitter
        // stdev (~3 deg) decouples camera orientation from the cube's exact
        // hypothesized position.
        let orbit_azimuth = uniform(0.0, 2.0 * PI) %= "orbit_azimuth";
        let orbit_sin_elevation = uniform(0.0, 1.0) %= "orbit_sin_elevation";
        let orbit_radius = uniform(0.1, 0.25) %= "orbit_radius";
        let lookat_yaw_jitter = normal(0.0, 0.05) %= "lookat_yaw_jitter";
        let lookat_pitch_jitter = normal(0.0, 0.05) %= "lookat_pitch_jitter";
        let x = orbit_camera(
            orbit_azimuth,
            orbit_sin_elevation,
            orbit_radius,
            [u, CUBE_HALF_EXTENT, v].into(),
            lookat_yaw_jitter,
            lookat_pitch_jitter,
        );

        let ground_c0 = uniform(0.0, 1.0) %= "ground_c0";
        let ground_c1 = uniform(0.0, 1.0) %= "ground_c1";
        let ground_c2 = uniform(0.0, 1.0) %= "ground_c2";
        let ground = (
            Box::new(Plane {
                origin: Vec3A::ZERO,
                normal: [0.0, 1.0, 0.0].into(),
            }) as Box<dyn Solid>,
            [ground_c0, ground_c1, ground_c2],
        );

        let scene = vec![ground, cube];
        let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());

        // depth + flat color observations, from one shared intersection pass
        // (see raytrace_depths_and_flat_colors -- avoids casting two full sets
        // of camera rays that would do identical ray setup and nearest-hit search)
        let mut depths = vec![0.0; AREA()];
        let mut colors = vec![[0.0; 3]; AREA()];
        if path_trace {
            raytrace_depths_and_path_colors(
                x,
                proj,
                &scene,
                [0.7, 0.7, 0.7],
                &mut depths,
                &mut colors,
            );
        } else {
            raytrace_depths_and_flat_colors(
                x,
                proj,
                &scene,
                [0.7, 0.7, 0.7],
                &mut depths,
                &mut colors,
            );
        }

        // illuminant/camera color cast: real captures are tinted by the light
        // source and the camera's white balance, which the renderer knows nothing
        // about. One multiplicative BGR latent absorbs most of that sim-to-real
        // color error (a full spectral reflectance model would only fix
        // second-order metamerism on top of this). Range keeps products in [0,1].
        let illum_c0 = uniform(0.5, 1.0) %= "illum_c0";
        let illum_c1 = uniform(0.5, 1.0) %= "illum_c1";
        let illum_c2 = uniform(0.5, 1.0) %= "illum_c2";
        for c in colors.iter_mut() {
            c[0] *= illum_c0;
            c[1] *= illum_c1;
            c[2] *= illum_c2;
        }

        noisy_depths(depths.clone(), depth_noise) %= "depth_observation";
        noisy_colors(colors.clone(), color_noise) %= "color_observation";

        (depths, colors)
    }
);
