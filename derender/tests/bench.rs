use glam::{Affine3A, Mat4, Vec3A};
pub use modppl_derender::{
    baseline,
    config::{AREA, FAR, FOVY, H, NEAR, W},
    core::*,
    serialization::save_depths,
};
use std::{fs::create_dir_all, time::Instant};

const NUM_FRAMES: usize = 10_000;

#[test]
pub fn test_compare_baseline_and_glam() {
    create_dir_all("out").expect("error creating 'out' dir");

    // Baseline
    let x = baseline::linear::vec3_euler_to_pose([0.0, 1.0, 1.2], [0.0, 0.0, 0.0]);
    let proj = baseline::linear::perspective(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
    let ground = (
        Box::new(baseline::solid::Plane {
            origin: baseline::linear::vec3_zero(),
            normal: [0.0, 1.0, 0.0],
        }) as Box<dyn baseline::solid::Solid>,
        [0.0, 0.0, 0.0],
    );
    let scene = vec![ground];

    let mut out = vec![0.0; AREA()];
    let now = Instant::now();
    for _ in 0..NUM_FRAMES {
        baseline::ray::raytrace_depths(x, proj, &scene, &mut out);
    }
    let fps1 = NUM_FRAMES as f64 / now.elapsed().as_secs_f64();
    println!("Baseline FPS: {fps1}",);
    save_depths("out/baseline_ground.png", &out);

    // Glam
    let x = Affine3A::from_translation([0.0, 1.0, 1.2].into());
    let proj = Mat4::perspective_rh_gl(FOVY(), W() as f32 / H() as f32, NEAR(), FAR());
    let ground =
        Box::new(Plane {
            origin: Vec3A::ZERO,
            normal: [0.0, 1.0, 0.0].into(),
        }) as Box<dyn Solid>;
    let scene = vec![ground];

    let mut out = vec![0.0; AREA()];
    let now = Instant::now();
    for _ in 0..NUM_FRAMES {
        raytrace_depths(x, proj, &scene, &mut out);
    }
    let fps2 = NUM_FRAMES as f64 / now.elapsed().as_secs_f64();
    println!("    Glam FPS: {fps2}");

    println!("Glam Perf Factor: x{}", fps2 / fps1);
    save_depths("out/glam_ground.png", &out);
}
