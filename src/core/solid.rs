use crate::image::*;

use glam::Vec3A;


pub trait Solid: Sync {
    fn ray_intersect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<f32>;
    fn ray_intersect_reflect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<(f32,Vec3A)>;

    /// Color at a hit, given the surface normal and the `Scene`-assigned `base_color`.
    /// Single-colored solids (the default) ignore the normal. Multi-colored solids
    /// like `Cube` override this to vary color by face.
    fn color_at(&self, _normal: Vec3A, base_color: Color) -> Color { base_color }
}

pub type Scene = Vec<(Box<dyn Solid>,Color)>;

pub struct Plane {
    pub origin: Vec3A,
    pub normal: Vec3A
}

impl Solid for Plane {
    fn ray_intersect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<f32> {
        let origin = self.origin;
        let normalv = self.normal;
        let denom = normalv.dot(ray_dir);
        let d = (origin - ray_origin).dot(normalv) / denom;
        if d > 1e-6 { Some(d) } else { None }
    }

    fn ray_intersect_reflect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<(f32,Vec3A)> {
        let origin = self.origin;
        let normalv = self.normal;
        let denom = normalv.dot(ray_dir);
        let d = (origin - ray_origin).dot(normalv) / denom;
        if d > 1e-6 { Some((d, normalv)) } else { None }
    }
}

pub struct Sphere {
    pub center: Vec3A,
    pub radius: f32
}

impl Sphere {
    fn intersect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<f32> {
        let mut dp = self.center - ray_origin;
        let ddp = ray_dir.dot(dp);
        let dpp = dp.dot(dp);
        
        // remedy term for numerical stability
        dp -= ray_dir * ddp;

        let r2 = self.radius * self.radius;
        let mut dscr = r2 - dp.dot(dp);

        if dscr < 0.0 {
            return None;
        }

        dscr = dscr.sqrt();
        let q = if ddp >= 0.0 { ddp + dscr } else { ddp - dscr };

        let mut t1 = q;
        let mut t2 = (dpp - r2) / q;

        if t1 > t2 { (t1, t2) = (t2, t1); }

        if t1 < 0.0 && t2 < 0.0 {
            return None;
        }

        Some(t1)
    }
}

impl Solid for Sphere {
    fn ray_intersect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<f32> {
        self.intersect(ray_origin, ray_dir)
    }

    fn ray_intersect_reflect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<(f32,Vec3A)> {
        if let Some(d) = self.intersect(ray_origin, ray_dir) {
            Some((d, ray_origin + ray_dir * d - self.center))
        } else {
            None
        }
    }
}

/// Finite cylinder aligned along +Y, with both end caps.
/// Extends from `base.y` to `base.y + height` (a mug standing on the ground).
pub struct Cylinder {
    pub base: Vec3A,
    pub radius: f32,
    pub height: f32
}

impl Cylinder {
    /// Returns the nearest positive hit as (distance, unit normal).
    fn intersect(&self, o: Vec3A, dir: Vec3A) -> Option<(f32,Vec3A)> {
        let eps = 1e-6;
        let r = self.radius;
        let y_lo = self.base.y;
        let y_hi = self.base.y + self.height;

        let dx = o.x - self.base.x;
        let dz = o.z - self.base.z;

        let mut best_t = f32::MAX;
        let mut best_n = Vec3A::ZERO;
        let mut hit = false;

        // side: infinite cylinder body, clamped to [y_lo, y_hi]
        let a = dir.x * dir.x + dir.z * dir.z;
        if a > eps {
            let b = 2.0 * (dx * dir.x + dz * dir.z);
            let c = dx * dx + dz * dz - r * r;
            let disc = b * b - 4.0 * a * c;
            if disc >= 0.0 {
                let sq = disc.sqrt();
                for t in [(-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a)] {
                    if t > eps && t < best_t {
                        let y = o.y + t * dir.y;
                        if y_lo <= y && y <= y_hi {
                            let px = o.x + t * dir.x - self.base.x;
                            let pz = o.z + t * dir.z - self.base.z;
                            best_t = t;
                            best_n = Vec3A::new(px, 0.0, pz) / r;
                            hit = true;
                        }
                    }
                }
            }
        }

        // end caps: disks at y_lo and y_hi
        if dir.y.abs() > eps {
            for (cap_y, n) in [(y_lo, Vec3A::new(0.0, -1.0, 0.0)), (y_hi, Vec3A::new(0.0, 1.0, 0.0))] {
                let t = (cap_y - o.y) / dir.y;
                if t > eps && t < best_t {
                    let px = o.x + t * dir.x - self.base.x;
                    let pz = o.z + t * dir.z - self.base.z;
                    if px * px + pz * pz <= r * r {
                        best_t = t;
                        best_n = n;
                        hit = true;
                    }
                }
            }
        }

        if hit { Some((best_t, best_n)) } else { None }
    }
}

impl Solid for Cylinder {
    fn ray_intersect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<f32> {
        self.intersect(ray_origin, ray_dir).map(|(t, _)| t)
    }

    fn ray_intersect_reflect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<(f32,Vec3A)> {
        self.intersect(ray_origin, ray_dir)
    }
}

/// Axis-aligned cube, colored like a Rubik's cube: each of the 6 faces gets a
/// fixed color (white/yellow/red/orange/blue/green) regardless of the
/// `Scene`-assigned `base_color`, which is ignored.
pub struct Cube {
    pub center: Vec3A,
    pub half_extent: f32
}

impl Cube {
    /// Returns the nearest positive hit as (distance, unit face normal), via the
    /// standard AABB slab method.
    fn intersect(&self, o: Vec3A, dir: Vec3A) -> Option<(f32,Vec3A)> {
        let eps = 1e-6;
        let he = Vec3A::splat(self.half_extent);
        let inv_dir = Vec3A::ONE / dir;

        let t1 = (self.center - he - o) * inv_dir;
        let t2 = (self.center + he - o) * inv_dir;
        let tmin = t1.min(t2);
        let tmax = t1.max(t2);

        let t_near = tmin.x.max(tmin.y).max(tmin.z);
        let t_far = tmax.x.min(tmax.y).min(tmax.z);

        if t_near > t_far || t_far < eps {
            return None;
        }
        let t = if t_near > eps { t_near } else { t_far };

        // determine which face was hit: the axis whose offset from center is
        // (proportionally) largest will sit at +/- half_extent
        let p = o + dir * t - self.center;
        let (ax, ay, az) = ((p.x / he.x).abs(), (p.y / he.y).abs(), (p.z / he.z).abs());
        let normal = if ax >= ay && ax >= az {
            Vec3A::new(p.x.signum(), 0.0, 0.0)
        } else if ay >= ax && ay >= az {
            Vec3A::new(0.0, p.y.signum(), 0.0)
        } else {
            Vec3A::new(0.0, 0.0, p.z.signum())
        };

        Some((t, normal))
    }
}

impl Solid for Cube {
    fn ray_intersect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<f32> {
        self.intersect(ray_origin, ray_dir).map(|(t, _)| t)
    }

    fn ray_intersect_reflect(&self, ray_origin: Vec3A, ray_dir: Vec3A) -> Option<(f32,Vec3A)> {
        self.intersect(ray_origin, ray_dir)
    }

    fn color_at(&self, normal: Vec3A, _base_color: Color) -> Color {
        // Color = [f32; 3] in BGR order (see image.rs). Standard Rubik's cube
        // scheme: white/yellow opposite, red/orange opposite, blue/green opposite.
        const WHITE:  Color = [1.000, 1.000, 1.000]; // +Y
        const YELLOW: Color = [0.000, 1.000, 1.000]; // -Y
        const RED:    Color = [0.000, 0.000, 0.835]; // -X
        const ORANGE: Color = [0.000, 0.349, 1.000]; // +X
        const BLUE:   Color = [0.792, 0.267, 0.000]; // +Z
        const GREEN:  Color = [0.282, 0.608, 0.000]; // -Z

        if normal.x > 0.5      { ORANGE }
        else if normal.x < -0.5 { RED }
        else if normal.y > 0.5  { WHITE }
        else if normal.y < -0.5 { YELLOW }
        else if normal.z > 0.5  { BLUE }
        else                    { GREEN }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    fn approx(a: f32, b: f32) -> bool { (a - b).abs() < 1e-4 }

    // unit cylinder: base at origin, radius 1, height 2 (spans y in [0, 2])
    fn unit() -> Cylinder {
        Cylinder { base: Vec3A::ZERO, radius: 1.0, height: 2.0 }
    }

    #[test]
    fn cylinder_hits_side() {
        // ray at y=1 traveling -Z from z=5 hits the side at z=1
        let (t, n) = unit()
            .intersect([0.0, 1.0, 5.0].into(), [0.0, 0.0, -1.0].into())
            .expect("expected a side hit");
        assert!(approx(t, 4.0), "t = {t}");
        assert!(approx(n.x, 0.0) && approx(n.y, 0.0) && approx(n.z, 1.0), "n = {n:?}");
        assert!(approx(n.length(), 1.0), "normal should be unit, got {}", n.length());
    }

    #[test]
    fn cylinder_hits_top_cap() {
        // ray straight down the axis hits the top cap at y=2
        let (t, n) = unit()
            .intersect([0.0, 5.0, 0.0].into(), [0.0, -1.0, 0.0].into())
            .expect("expected a cap hit");
        assert!(approx(t, 3.0), "t = {t}");
        assert!(approx(n.y, 1.0), "n = {n:?}");
    }

    #[test]
    fn cylinder_misses_above_height() {
        // ray crosses the infinite cylinder but above the finite top (y=5 > 2)
        assert!(unit()
            .intersect([0.0, 5.0, 5.0].into(), [0.0, 0.0, -1.0].into())
            .is_none());
    }

    #[test]
    fn cylinder_misses_radially() {
        // ray stays outside the radius for all t
        assert!(unit()
            .intersect([5.0, 1.0, 5.0].into(), [0.0, 0.0, -1.0].into())
            .is_none());
    }

    // unit cube: centered at origin, half-extent 1 (spans [-1, 1] on each axis)
    fn unit_cube() -> Cube {
        Cube { center: Vec3A::ZERO, half_extent: 1.0 }
    }

    #[test]
    fn cube_hits_top_face() {
        let (t, n) = unit_cube()
            .intersect([0.0, 5.0, 0.0].into(), [0.0, -1.0, 0.0].into())
            .expect("expected a top-face hit");
        assert!(approx(t, 4.0), "t = {t}");
        assert!(approx(n.x, 0.0) && approx(n.y, 1.0) && approx(n.z, 0.0), "n = {n:?}");
    }

    #[test]
    fn cube_hits_side_face() {
        // ray along -X hits the +X face first
        let (t, n) = unit_cube()
            .intersect([5.0, 0.0, 0.0].into(), [-1.0, 0.0, 0.0].into())
            .expect("expected a side-face hit");
        assert!(approx(t, 4.0), "t = {t}");
        assert!(approx(n.x, 1.0) && approx(n.y, 0.0) && approx(n.z, 0.0), "n = {n:?}");
    }

    #[test]
    fn cube_misses() {
        // ray passes well outside the cube
        assert!(unit_cube()
            .intersect([5.0, 5.0, 5.0].into(), [0.0, 0.0, -1.0].into())
            .is_none());
    }

    #[test]
    fn cube_face_colors_are_distinct_and_match_rubiks_scheme() {
        let cube = unit_cube();
        let dummy_base = [0.0, 0.0, 0.0];

        let colors = [
            cube.color_at([1.0, 0.0, 0.0].into(), dummy_base),  // +X orange
            cube.color_at([-1.0, 0.0, 0.0].into(), dummy_base), // -X red
            cube.color_at([0.0, 1.0, 0.0].into(), dummy_base),  // +Y white
            cube.color_at([0.0, -1.0, 0.0].into(), dummy_base), // -Y yellow
            cube.color_at([0.0, 0.0, 1.0].into(), dummy_base),  // +Z blue
            cube.color_at([0.0, 0.0, -1.0].into(), dummy_base), // -Z green
        ];

        // all 6 face colors should be distinct
        for i in 0..colors.len() {
            for j in (i+1)..colors.len() {
                assert_ne!(colors[i], colors[j], "faces {i} and {j} have the same color");
            }
        }

        // base_color should be ignored entirely (cube is multi-colored, not scene-tinted)
        assert_eq!(cube.color_at([1.0, 0.0, 0.0].into(), [0.5, 0.5, 0.5]), colors[0]);
    }
}