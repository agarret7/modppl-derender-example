//! Shared pixel likelihoods, reused across scene models the same way
//! `core::proposals` are -- these are general-purpose building blocks, not
//! "a model" themselves, so they stay here rather than moving out to
//! `examples/scenes` with the actual generative functions.

use float_extras::f64::erf;
use modppl::prelude::*;

use crate::config::*;
use crate::image::*;

/// truncated Gaussian distribution type
pub struct TruncatedNormal {}
pub const truncated_normal: TruncatedNormal = TruncatedNormal {};

// erf has no f32 variant in float_extras (it's an FFI wrapper over libm's
// double-precision erf), so the CDF stays in f64 internally regardless of
// modppl's Real precision; only the final logpdf crosses back to f32.
fn normal_cdf(x: &f64, params: (f64, f64)) -> f64 {
    let (mu, sigma) = params;
    let xi = (x - mu) / sigma;
    0.5 * (1. + erf(xi / 2f64.sqrt()))
}

impl Distribution<f32, (f32, f32, f32, f32)> for TruncatedNormal {
    fn logpdf(&self, x: &f32, params: (f32, f32, f32, f32)) -> f32 {
        let (mu, sigma, a, b) = params;
        if a <= *x && *x <= b {
            let log_cdf_range = (normal_cdf(&(b as f64), (mu as f64, sigma as f64))
                - normal_cdf(&(a as f64), (mu as f64, sigma as f64)))
            .ln();
            // normal.logpdf already includes the -ln(sigma) normalization;
            // truncation only adds the -ln(Z) renormalizer.
            normal.logpdf(x, (mu, sigma)) - log_cdf_range as f32
        } else {
            f32::NEG_INFINITY
        }
    }

    fn random(&self, rng: &mut ThreadRng, params: (f32, f32, f32, f32)) -> f32 {
        let (mu, sigma, a, b) = params;
        let mut x = normal.random(rng, (mu, sigma));
        while !(a <= x && x <= b) {
            // rejection sampling
            x = normal.random(rng, (mu, sigma));
        }
        x
    }
}

/// noisy depth distribution type
pub struct NoisyDepths {}
pub const noisy_depths: NoisyDepths = NoisyDepths {};

impl Distribution<Depths, (Depths, f32)> for NoisyDepths {
    fn logpdf(&self, noisy_pixels: &Depths, pixels_and_noise: (Depths, f32)) -> f32 {
        let (pixels, noise) = pixels_and_noise;
        let mut w = 0.;
        for y in 0..H() {
            for x in 0..W() {
                let noisy_p = noisy_pixels[y * W() + x];
                // NaN marks a real sensor dropout (no measurement, see
                // realsense::depths_from_frame) -- not a real value, so it
                // carries no evidence and must be excluded, not scored as if
                // it meant "background."  Synthetic renders never produce NaN.
                if noisy_p.is_nan() {
                    continue;
                }
                let true_p = pixels[y * W() + x];
                w += truncated_normal.logpdf(&noisy_p, (true_p, noise, 0.0, 1.0))
            }
        }
        (1. - noise) * w
    }

    fn random(&self, rng: &mut ThreadRng, pixels_and_noise: (Depths, f32)) -> Depths {
        let (pixels, noise) = pixels_and_noise;
        let mut noisy_pixels = vec![];
        for y in 0..H() {
            for x in 0..W() {
                // Add mixture of noise from uniform and gaussian
                if u01(rng) < noise {
                    noisy_pixels.push(u01(rng));
                } else {
                    let noisy_p =
                        truncated_normal.random(rng, (pixels[y * W() + x], noise, 0.0, 1.0));
                    noisy_pixels.push(noisy_p);
                }
            }
        }
        noisy_pixels
    }
}

/// noisy (isotropic) color distribution type
pub struct NoisyColors {}
pub const noisy_colors: NoisyColors = NoisyColors {};

impl Distribution<Colors, (Colors, f32)> for NoisyColors {
    fn logpdf(&self, noisy_pixels: &Colors, pixels_and_noise: (Colors, f32)) -> f32 {
        let (pixels, noise) = pixels_and_noise;
        let mut w = 0.;
        for y in 0..H() {
            for x in 0..W() {
                for i in 0..=2 {
                    let noisy_p = noisy_pixels[y * W() + x][i];
                    let true_p = pixels[y * W() + x][i];
                    w += truncated_normal.logpdf(&noisy_p, (true_p, noise, 0.0, 1.0))
                }
            }
        }
        (1. - noise) * w
    }

    fn random(&self, rng: &mut ThreadRng, pixels_and_noise: (Colors, f32)) -> Colors {
        let (pixels, noise) = pixels_and_noise;
        let mut noisy_pixels = vec![];
        for y in 0..H() {
            for x in 0..W() {
                // Add mixture of noise from uniform and gaussian
                let mut noisy_p = [0.0; 3];
                for i in 0..=2 {
                    if u01(rng) < noise {
                        noisy_p[i] = u01(rng);
                    } else {
                        noisy_p[i] =
                            truncated_normal.random(rng, (pixels[y * W() + x][i], noise, 0., 1.));
                    }
                }
                noisy_pixels.push(noisy_p)
            }
        }
        noisy_pixels
    }
}
