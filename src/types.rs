use crate::linear::*;


/* types */

pub type Mat4 = [[f32; 4]; 4];
pub type Vec3 = [f32; 3];
pub type Vec4 = [f32; 4];
pub type Quat = Vec4;
pub type Pose = [f32; 7];

pub type Depth =  f32;      // in [0.0, 1.0]
pub type Color = [f32; 3];  // in [0.0, 1.0] (BGR)

pub type Depths = Vec<Depth>;
pub type Colors = Vec<Color>;


pub trait Solid {
    fn ray_intersect(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<f32>;
    fn ray_intersect_reflect(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<(f32,Vec3)>;
}

pub type Scene = Vec<(Box<dyn Solid>,Color)>;

pub struct Plane {
    pub origin: Vec3,
    pub normal: Vec3
}

impl Solid for Plane {
    fn ray_intersect(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<f32> {
        let origin = self.origin;
        let normalv = self.normal;
        let denom = vec3_dot(&normalv, &ray_dir);
        let d = vec3_dot(&vec3_sub(origin, ray_origin), &normalv) / denom;
        if d > 1e-6 { Some(d) } else { None }
    }

    fn ray_intersect_reflect(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<(f32,Vec3)> {
        let origin = self.origin;
        let normalv = self.normal;
        let denom = vec3_dot(&normalv, &ray_dir);
        let d = vec3_dot(&vec3_sub(origin, ray_origin), &normalv) / denom;
        if d > 1e-6 { Some((d, normalv)) } else { None }
    }
}

pub struct Sphere {
    pub center: Vec3,
    pub radius: f32
}

impl Sphere {
    fn intersect(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<f32> {
        let mut dp = vec3_sub(self.center, ray_origin);
        let ddp = vec3_dot(&ray_dir, &dp);
        let dpp = vec3_norm_squared(&dp);
        
        // remedy term for numerical stability
        vec3_mulsubs(ray_dir, ddp, &mut dp);

        let r2 = self.radius * self.radius;
        let mut dscr = r2 - vec3_norm_squared(&dp);

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
    fn ray_intersect(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<f32> {
        self.intersect(ray_origin, ray_dir)
    }

    fn ray_intersect_reflect(&self, ray_origin: Vec3, ray_dir: Vec3) -> Option<(f32,Vec3)> {
        if let Some(d) = self.intersect(ray_origin, ray_dir) {
            Some((d, vec3_sub(ray_at(ray_origin, ray_dir, d), self.center)))
        } else {
            None
        }
    }
}
