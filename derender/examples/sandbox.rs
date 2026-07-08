//! The scene sandbox — one entrypoint, every demo scene, uniform CLI.
//!
//!   cargo run --release --example sandbox -- <scene>
//!
//! Scenes (each lives in `examples/scenes/<name>.rs` — open the file, edit
//! the model or the kernel, rerun):
//!
//!   ground       depth-only camera pose vs. a ground plane
//!   sphere       sphere position + color under unknown lighting
//!   ball         a ball on a table; test_derender_ball derenders a real photo
//!   mug          a cylinder with unknown pose, size, and color
//!   cone_sphere  two objects; inference must disentangle their pixels
//!   rubiks       the full real-world cube pipeline (metric scale, RGB-D),
//!                with extension ideas and a commented live-RealSense swap
//!
//! All windows share the harness keys: Space=pause, R=fresh scene,
//! S=snapshot, ESC=quit.

#[path = "scenes/ball.rs"]
mod ball;
#[path = "scenes/cone_sphere.rs"]
mod cone_sphere;
#[path = "scenes/ground.rs"]
mod ground;
#[path = "scenes/mug.rs"]
mod mug;
#[path = "scenes/rubiks.rs"]
mod rubiks;
#[path = "scenes/sphere.rs"]
mod sphere;

const USAGE: &str = "\
usage: cargo run --release --example sandbox -- <scene>

scenes:
  ground       depth-only camera pose vs. a ground plane
  sphere       sphere position + color under unknown lighting
  ball         a ball on a table; test_derender_ball derenders a real photo
  mug          a cylinder with unknown pose, size, and color
  cone_sphere  two objects; inference must disentangle their pixels
  rubiks       the full real-world cube pipeline (metric scale, RGB-D)";

fn main() {
    match std::env::args().nth(1).as_deref() {
        Some("ground") => ground::run(),
        Some("sphere") => sphere::run(),
        Some("ball") => ball::run(),
        Some("mug") => mug::run(),
        Some("cone_sphere") => cone_sphere::run(),
        Some("rubiks") => rubiks::run(),
        Some(other) => {
            eprintln!("unknown scene: {other}\n\n{USAGE}");
            std::process::exit(1);
        }
        None => {
            eprintln!("{USAGE}");
            std::process::exit(1);
        }
    }
}
