use std::f32::consts::PI;
use modppl::prelude::*;
use glam::{Mat4, Affine3A, Vec3A, Vec4};
use rayon::prelude::*;

use crate::config::{H, W, NEAR, FAR, VP};
use crate::image::*;
use crate::core::solid::Scene;


/* constants */


struct UniformS2 { }
const uniform_s2: UniformS2 = UniformS2 { };

impl Distribution<Vec3A,()> for UniformS2 {
    fn logpdf(&self, _: &Vec3A, _: ()) -> f64 {
        panic!("not implemented!");
    }
    
    fn random(&self, rng: &mut ThreadRng, _: ()) -> Vec3A {
        let z = 2.0 * u01(rng) as f32 - 1.0;
        let theta = 2.0 * PI * u01(rng) as f32;
        let r = (1.0 - z * z).sqrt();
        [r * theta.cos(), r * theta.sin(), z].into()
    }
}


fn view_to_ndch(pos: &Vec3A, vp: &[f32; 4]) -> Vec4 {
    let mut v = Vec4::ZERO;
    v[0] = 2.0 * (pos[0] - vp[0]) / vp[2] - 1.0;
    v[1] = 2.0 * (pos[1] - vp[1]) / vp[3] - 1.0;
    v[2] = pos[2];
    v[3] = 1.0;
    v
}

// /* cpu ray tracers */

/// returns a depth raytace
pub fn raytrace_depths(x: Affine3A, proj: Mat4, scene: &Scene, out: &mut Depths) {
    let i = x * proj.inverse();

    for y in 0..H() {
        for x in 0..W() {
            let near_ndch = view_to_ndch(&[x as f32, (H() - y) as f32, -1.0].into(), &VP());
            let far_ndch = view_to_ndch(&[x as f32, (H() - y) as f32,  1.0].into(), &VP());

            let near_pw = i * near_ndch;
            let far_pw = i * far_ndch;
            let near_pw: Vec3A = (near_pw / near_pw[3]).truncate().into();
            let far_pw: Vec3A = (far_pw / far_pw[3]).truncate().into();

            let ray_origin = near_pw;
            let ray_dir = (far_pw - near_pw).normalize();

            for s in scene.iter() {
                if let Some(d) = s.0.ray_intersect(ray_origin, ray_dir) {
                    let lm = if NEAR() <= d && d <= FAR() {
                        1.0 - (d - NEAR()) / (FAR() - NEAR())
                    } else {
                        0.0
                    };
                    out[y * W() + x] = lm;
                }
            }
        }
    }
}

/// single-sample, no-lighting "flat" color raytrace: returns each hit solid's
/// `color_at` (e.g. `Cube`'s fixed per-face label color) directly, with no
/// shading or bounces. Intended for objects with deliberately flat, highly
/// saturated surfaces (e.g. a Rubik's cube's stickers), where simulating
/// realistic lighting would hurt rather than help sim-to-real color matching.
pub fn raytrace_flat_colors(x: Affine3A, proj: Mat4, scene: &Scene, background_color: Color, out: &mut Colors) {
    let i = x * proj.inverse();

    out.par_chunks_mut(W()).enumerate().for_each(|(y, row)| {
        for x in 0..W() {
            let u = x as f32 + 0.5;
            let v = (H() - y) as f32 - 0.5;

            let near_ndch = view_to_ndch(&[u, v, -1.0].into(), &VP());
            let far_ndch = view_to_ndch(&[u, v,  1.0].into(), &VP());

            let near_pw = i * near_ndch;
            let far_pw = i * far_ndch;
            let near_pw: Vec3A = (near_pw / near_pw[3]).truncate().into();
            let far_pw: Vec3A = (far_pw / far_pw[3]).truncate().into();

            let ray_origin = near_pw;
            let ray_dir = (far_pw - near_pw).normalize();

            let mut distance = f32::MAX;
            let mut hit = false;
            let mut normalv = Vec3A::ZERO;
            let mut hit_idx = 0;

            for (idx, s) in scene.iter().enumerate() {
                if let Some((d, n)) = s.0.ray_intersect_reflect(ray_origin, ray_dir) {
                    if d < distance {
                        hit = true;
                        distance = d;
                        normalv = n;
                        hit_idx = idx;
                    }
                }
            }

            row[x] = if hit {
                scene[hit_idx].0.color_at(normalv, scene[hit_idx].1)
            } else {
                background_color
            };
        }
    });
}

/// returns a color raytrace with diffuse (Lambertian) reflection and global illumination
pub fn raytrace_colors(x: Affine3A, proj: Mat4, scene: &Scene, background_color: Color, out: &mut Colors) {
    let i = x * proj.inverse();

    let num_samples = 10;
    let cnorm = 1.0 / num_samples as f32;
    let max_depth = 10;

    out.par_chunks_mut(W()).enumerate().for_each(|(y, row)| {
        let mut rng = ThreadRng::default();
        for x in 0..W() {
            let total_c = (0..num_samples).map(|_| {
                let mut c = [0.0; 3];
                let mut cs = [0.0; 3];
                let u = x as f32 + u01(&mut rng) as f32 + 0.5;
                let v = (H() - y) as f32 + u01(&mut rng) as f32 - 0.5;

                let near_ndch = view_to_ndch(&[u, v, -1.0].into(), &VP());
                let far_ndch = view_to_ndch(&[u, v,  1.0].into(), &VP());

                let near_pw = i * near_ndch;
                let far_pw = i * far_ndch;
                let near_pw: Vec3A = (near_pw / near_pw[3]).truncate().into();
                let far_pw: Vec3A = (far_pw / far_pw[3]).truncate().into();

                let mut ray_origin = near_pw;
                let mut ray_dir = far_pw - near_pw;

                let mut depth = max_depth;
                let mut transmittance = [1.0; 3];
                while depth > 0 {
                    ray_dir = ray_dir.normalize();

                    let mut distance = f32::MAX;
                    let mut hit = false;
                    let mut normalv = Vec3A::ZERO;
                    let mut hit_idx = 0;

                    for (idx, s) in scene.iter().enumerate() {
                        if let Some((d, n)) = s.0.ray_intersect_reflect( ray_origin, ray_dir) {
                            if d < distance {
                                hit = true;
                                distance = d;
                                normalv = n;
                                hit_idx = idx;
                            }
                        }
                    }

                    if hit {
                        cs = scene[hit_idx].0.color_at(normalv, scene[hit_idx].1);
                        ray_origin = ray_origin + ray_dir * distance;
                        ray_dir = normalv + uniform_s2.random(&mut rng, ());

                        transmittance[0] *= cs[0];
                        transmittance[1] *= cs[1];
                        transmittance[2] *= cs[2];

                        depth -= 1;
                    } else {
                        c[0] += transmittance[0] * background_color[0];
                        c[1] += transmittance[1] * background_color[1];
                        c[2] += transmittance[2] * background_color[2];
                        break;
                    }
                }
                c
            }).fold([0.0; 3], |a, c| [a[0] + c[0], a[1] + c[1], a[2] + c[2]]);

            // normalize and apply gamma correction
            let finv_gamma = 0.5;
            row[x][0] = (total_c[0]*cnorm).powf(finv_gamma);
            row[x][1] = (total_c[1]*cnorm).powf(finv_gamma);
            row[x][2] = (total_c[2]*cnorm).powf(finv_gamma);
        }
    });
}