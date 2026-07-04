use std::f32::consts::PI;
use float_extras::f64::erf;
use modppl::prelude::*;

use crate::baseline::linear::*;
use crate::baseline::solid::*;
use crate::baseline::ray::*;
use crate::config::*;
use crate::image::*;


/// truncated Gaussian distribution type
pub struct TruncatedNormal { }
pub const truncated_normal: TruncatedNormal = TruncatedNormal { };

// erf has no f32 variant in float_extras; the CDF stays f64 internally
// regardless of modppl's Real precision, only the final logpdf crosses back.
fn normal_cdf(x: &f64, params: (f64,f64)) -> f64 {
    let (mu, sigma) = params;
    let xi = (x - mu) / sigma;
    0.5*(1. + erf(xi/2f64.sqrt()))
}

impl Distribution<f32,(f32,f32,f32,f32)> for TruncatedNormal {
    fn logpdf(&self, x: &f32, params: (f32,f32,f32,f32)) -> f32 {
        let (mu, sigma, a, b) = params;
        if a <= *x && *x <= b {
            let log_cdf_range = (normal_cdf(&(b as f64), (mu as f64, sigma as f64)) -
                                  normal_cdf(&(a as f64), (mu as f64, sigma as f64))).ln();
            normal.logpdf(x, (mu, sigma)) - log_cdf_range as f32 - sigma.ln()
        } else {
            f32::NEG_INFINITY
        }
    }

    fn random(&self, rng: &mut ThreadRng, params: (f32,f32,f32,f32)) -> f32 {
        let (mu, sigma, a, b) = params;
        let mut x = normal.random(rng, (mu, sigma));
        while !(a <= x && x <= b) {  // rejection sampling
            x = normal.random(rng, (mu, sigma));
        }
        x
    }
}

/* pixel likelihoods */

/// noisy depth distribution type
struct NoisyDepths { }
const noisy_depths: NoisyDepths = NoisyDepths { };

impl Distribution<Depths,(Depths,f32)> for NoisyDepths {
    fn logpdf(&self, noisy_pixels: &Depths, pixels_and_noise: (Depths,f32)) -> f32 {
        let (pixels, noise) = pixels_and_noise;
        let mut w = 0.;
        for y in 0..H() {
            for x in 0..W() {
                let noisy_p = noisy_pixels[y*W() + x];
                let true_p = pixels[y*W() + x];
                w += truncated_normal.logpdf(&noisy_p, (true_p, noise, 0.0, 1.0))
            }
        }
        (1. - noise) * w
    }

    fn random(&self, rng: &mut ThreadRng, pixels_and_noise: (Depths,f32)) -> Depths {
        let (pixels, noise) = pixels_and_noise;
        let mut noisy_pixels = vec![];
        for y in 0..H() {
            for x in 0..W() {
                // Add mixture of noise from uniform and gaussian
                if u01(rng) < noise {
                    noisy_pixels.push(u01(rng));
                } else {
                    let noisy_p = truncated_normal.random(rng, (pixels[y*W() + x], noise, 0.0, 1.0));
                    noisy_pixels.push(noisy_p);
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
    fn logpdf(&self, noisy_pixels: &Colors, pixels_and_noise: (Colors,f32)) -> f32 {
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
        (1. - noise) * w
    }

    fn random(&self, rng: &mut ThreadRng, pixels_and_noise: (Colors,f32)) -> Colors {
        let (pixels, noise) = pixels_and_noise;
        let mut noisy_pixels = vec![];
        for y in 0..H() {
            for x in 0..W() {
                // Add mixture of noise from uniform and gaussian
                let mut noisy_p = vec3_zero();
                for i in 0..=2 {
                    if u01(rng) < noise {
                        noisy_p[i] = u01(rng);
                    } else {
                        noisy_p[i] = truncated_normal.random(rng, (pixels[y*W() + x][i], noise, 0., 1.));
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
    let cam_roll = normal(0.0, PI/8.0) %= "cam_roll";
    let x = vec3_euler_to_pose([0.0, cam_y, 1.2], [0.0, 0.0, cam_roll]);

    // ground
    let ground = (
        Box::new(Plane { origin: vec3_zero(), normal: [0.0, 1.0, 0.0] }) as Box<dyn Solid>,
        [0.0, 0.0, 0.0]
    );

    // render
    let proj = perspective(FOVY(), W() as f32/H() as f32, NEAR(), FAR());
    let mut pixels = vec![0.0; AREA()];
    raytrace_depths(x, proj, &vec![ground], &mut pixels);
    noisy_depths(pixels.clone(), 0.1) %= "observation";

    pixels
});

dyngen!(
pub fn sphere_color_model() -> Colors {
    // camera pose
    let cam_y = uniform(0.5, 2.0) %= "cam_y";
    let cam_roll = normal(0.0, PI/8.0) %= "cam_roll";
    let x = vec3_euler_to_pose([0.0, cam_y, 1.2], [0.0, 0.0, cam_roll]);

    // background
    let brightness = uniform(0.5, 1.0) %= "ambient_brightness";
    let background_color = [brightness, brightness, brightness];

    // ground
    let ground_albedo = uniform(0.0, 1.0) %= "ground_albedo";
    let ground = (
        Box::new(Plane { origin: vec3_zero(), normal: [0.0, 1.0, 0.0] }) as Box<dyn Solid>,
        [ground_albedo, ground_albedo, ground_albedo]
    );

    // sphere
    let u = uniform(-2.0, 2.0) %= "sphere_u";
    let v = uniform(-2.0, 0.0) %= "sphere_v";
    let redness = uniform(0.0, 1.0) %= "sphere_redness";
    let sphere = (
        Box::new(Sphere { center: [u, 0.5, v], radius: 0.5 }) as Box<dyn Solid>,
        [0.2, 1.0 - redness, redness]
    );

    // render
    let proj = perspective(FOVY(), W() as f32/H() as f32, NEAR(), FAR());
    let mut pixels = vec![[0.0; 3]; AREA()];
    raytrace_colors(x, proj, &vec![ground, sphere], background_color, &mut pixels);
    noisy_colors(pixels.clone(), 0.1) %= "observation";

    pixels
});

dyngen!(
pub fn ball_model() -> Colors {
    // camera pose
    let cam_y = uniform(0.5, 2.0) %= "cam_y";
    let cam_yaw = normal(0.0, PI/8.0) %= "cam_yaw";
    let x = vec3_euler_to_pose([0.0, cam_y, 1.2], [cam_yaw, 0.0, 0.0]);

    // background
    let brightness = uniform(0.75, 1.0) %= "ambient_brightness";
    let global_c = vec3_scale(&[0.9, 1.0, 1.0], brightness);
    let background_color = global_c;

    // table
    let mut table_c = vec3_zero();
    table_c[0] = uniform(0.0, 1.0) %= "table_c0";
    table_c[1] = uniform(0.0, 1.0) %= "table_c1";
    table_c[2] = uniform(0.0, 1.0) %= "table_c2";
    let table = (
        Box::new(Plane { origin: vec3_zero(), normal: [0.0, 1.0, 0.0] }) as Box<dyn Solid>,
        table_c
    );

    // ball
    let u = uniform(-1.0, 1.0) %= "ball_u";
    let v = uniform(-1.0, 0.0) %= "ball_v";
    let mut ball_c = vec3_zero();
    ball_c[0] = uniform(0.25, 1.0) %= "ball_c0";
    ball_c[1] = uniform(0.25, 1.0) %= "ball_c1";
    ball_c[2] = uniform(0.25, 1.0) %= "ball_c2";
    let ball_r = uniform(0.3, 0.5) %= "ball_radius";
    let ball = (
        Box::new(Sphere { center: [u, ball_r, v], radius: ball_r }) as Box<dyn Solid>,
        ball_c
    );

    // render
    let proj = perspective(FOVY(), W() as f32/H() as f32, NEAR(), FAR());
    let mut pixels = vec![[0.0; 3]; AREA()];
    raytrace_colors(x, proj, &vec![table, ball], background_color, &mut pixels);
    noisy_colors(pixels.clone(), 0.1) %= "observation";

    pixels
});

dyngen!(
pub fn gaussian_drift(trace: Weak<DynTrace<(),Colors>>, mask: Vec<&str>, stdev: f32) {
    let trace = trace.upgrade().unwrap();
    for addr in mask.iter() {
        normal(trace.data.read::<f32>(addr), stdev) %= addr;
    }
});