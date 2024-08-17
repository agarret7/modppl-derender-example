use std::f32::consts::PI;
use modppl::prelude::*;

use crate::types::*;
use crate::linear::*;
use crate::{NEAR,FAR};


/* constants */

pub const H   : usize = 128;
pub const W   : usize = 128;
pub const AREA: usize = H*W;


/* cpu ray tracers */

struct UniformS2 { }
const uniform_s2: UniformS2 = UniformS2 { };

impl Distribution<Vec3,()> for UniformS2 {
    fn logpdf(&self, _: &Vec3, _: ()) -> f64 {
        panic!("not implemented!");
    }
    
    fn random(&self, rng: &mut ThreadRng, _: ()) -> Vec3 {
        let z = 2.0 * u01(rng) as f32 - 1.0;
        let theta = 2.0 * PI * u01(rng) as f32;
        let r = (1.0 - z * z).sqrt();
        [r * theta.cos(), r * theta.sin(), z]
    }
}

/// returns a depth raytace
pub fn raytrace_depths(x: Pose, proj: Mat4, scene: &Scene, out: &mut Depths) {
    let iso = pose_to_mat4(x);
    let i_proj = mat4_inv(proj);
    let i = mat4_mul(iso, i_proj);

    for y in 0..H {
        for x in 0..W {
            let vp = [ 0.0, 0.0, W as f32, H as f32 ];
            let near_pw = unproject_inv([x as f32, (H - y) as f32, -1.0], i, vp);
            let far_pw  = unproject_inv([x as f32, (H - y) as f32,  1.0], i, vp);

            let ray_origin = near_pw;
            let mut ray_dir = vec3_sub(far_pw, near_pw);
            vec3_normalize(&mut ray_dir);

            for s in scene.iter() {
                if let Some(d) = s.0.ray_intersect(ray_origin, ray_dir) {
                    let lm = if NEAR <= d && d <= FAR {
                        1.0 - (d - NEAR) / (FAR - NEAR)
                    } else {
                        0.0
                    };
                    out[y * W + x] = lm;
                }
            }
        }
    }
}

/// returns a color raytrace with diffuse (Lambertian) reflection and global illumination
pub fn raytrace_colors(x: Pose, proj: Mat4, scene: &Scene, background_color: Color, out: &mut Colors) {
    let mut rng = ThreadRng::default();

    let iso = pose_to_mat4(x);
    let i_proj = mat4_inv(proj);
    let i = mat4_mul(iso, i_proj);

    let num_samples = 10;
    let vp = [ 0.0, 0.0, W as f32, H as f32 ];
    let cnorm = 1.0 / num_samples as f32;
    let max_depth = 10;

    for y in 0..H {
        for x in 0..W {
            let total_c = (0..num_samples).map(|_| {
                let mut c = vec3_zero();
                let mut cs = vec3_zero();
                // the one place we add sampling INTERNAL to the ray-tracer: dithering
                let u = x as f32 + u01(&mut rng) as f32 + 0.5;
                let v = (H - y) as f32 + u01(&mut rng) as f32 - 0.5;
                let near_pw = unproject_inv([u, v, -1.0], i, vp);
                let far_pw  = unproject_inv([u, v,  1.0], i, vp);

                let mut ray_origin = near_pw;
                let mut ray_dir = vec3_sub(far_pw, near_pw);

                let mut depth = max_depth;
                let mut transmittance = [1.0; 3];
                while depth > 0 {
                    vec3_normalize(&mut ray_dir);

                    let mut distance = f32::MAX;
                    let mut hit = false;
                    let mut normalv = vec3_zero();

                    for s in scene.iter() {
                        if let Some((d, n)) = s.0.ray_intersect_reflect( ray_origin, ray_dir) {
                            if d < distance {
                                hit = true;
                                distance = d;
                                normalv = n;
                                cs = s.1;
                            }
                        }
                    }

                    if hit {
                        ray_origin = ray_at(ray_origin, ray_dir, distance);
                        ray_dir = vec3_add(normalv, uniform_s2.random(&mut rng, ()));

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
            }).fold(vec3_zero(), |a, c| [a[0] + c[0], a[1] + c[1], a[2] + c[2]]);

            // normalize and apply gamma correction
            let finv_gamma = 0.5;
            out[y * W + x][0] = (total_c[0]*cnorm).powf(finv_gamma);
            out[y * W + x][1] = (total_c[1]*cnorm).powf(finv_gamma);
            out[y * W + x][2] = (total_c[2]*cnorm).powf(finv_gamma);
        }
    }
}