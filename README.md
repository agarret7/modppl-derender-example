# modppl-derender

Inverse graphics in [ModPPL](https://crates.io/crates/modppl): combine a rendering-based
generative scene model with user-space MCMC inference to recover structured 3D scene
hypotheses directly from images. Each scene is a probabilistic program (a prior over camera
pose, object pose/size/color, and lighting) paired with a ray-traced renderer used as the
observation likelihood. Inference is plain Metropolis-Hastings over the program's own random
choices, with custom proposals where the default ones aren't enough.

## Gallery

Each pair below is observation (left) vs. the inferred hypothesis converging onto it (right),
20fps. Click a thumbnail for the full video.

### Synthetic

| | | |
|---|---|---|
| [![ground](out/thumbs/ground.png)](out/ground.mp4)<br>**Depth-only ground plane** — camera pose from a depth observation alone | [![sphere](out/thumbs/sphere1.png)](out/sphere1.mp4)<br>**Sphere + color** — position, ground albedo, ambient light, and sphere hue | [![mug](out/thumbs/mug.png)](out/mug.mp4)<br>**Mug** — a cylinder with unknown pose and color, more latent dimensions than the ball |
| [![rubiks](out/thumbs/rubiks.png)](out/rubiks.mp4)<br>**Rubik's cube** — a cube with fixed, deterministic per-face colors (`Cube::color_at`), only pose/size inferred | | |

### Real hardware

| | |
|---|---|
| [![ball](out/thumbs/ball.png)](out/ball.mp4)<br>**Ball, from a real photo** — derendering an actual `.bmp` photograph, not a synthetic render | [![live rgbd](out/thumbs/live_rgbd.png)](out/thumbs/live_rgbd.png)<br>**Live RealSense demo** — real depth+color camera input (top) vs. the live-inferred cube hypothesis (bottom), via `live_rgbd_cube` |

## Models

- `grounded_depth_model` — depth-only camera pose recovery against a ground plane.
- `sphere_color_model` — sphere position + color under unknown lighting.
- `ball_model` — a ball on a table; the `test_derender_ball` test derenders an actual
  photograph (`tests/ball.bmp`), not a synthetic observation.
- `mug_model` — a cylinder (mug) with unknown pose, size, and color.
- `cone_sphere_model` — a cone and a sphere, each with independent position, size, and
  color, on a shared table. The first multi-object scene: inference has to disentangle
  which pixels belong to which object.
- `cube_rgbd_model` — **the one Rubik's cube model, end-to-end**: real-world metric scale
  (fixed 4.5cm cube, orbital camera at 0.25–0.7m), joint depth+color observations, and a
  `path_trace` flag selecting flat rendering (deterministic — required for MCMC likelihoods
  and CNN training data) or path-traced rendering (for visuals). Used unchanged by the
  synthetic test, the interactive window, the live RealSense demo, and the pose-net
  training dataset.

All live in [`derender/src/core/models.rs`](derender/src/core/models.rs), written with the
`dyngen!` macro. Inference sweeps are built with the `Kernel` combinator
([`derender/src/inference.rs`](derender/src/inference.rs)):

```rust
let renders: Vec<Colors> = Kernel::new(&ball_model, trace)
    .regen_mh(&cam_mask)
    .regen_mh(&env_mask)
    .mh(&gaussian_drift, (vec!["table_c0", "table_c1", "table_c2"], 0.1))
    .regen_mh(&ball_mask)
    .mh(&gaussian_drift, (vec!["ball_u", "ball_v", "ball_radius"], 0.1))
    .regen_mh(&ball_color_mask)
    .take(NUM_ITERS)
    .map(|t| t.retv.clone().unwrap())
    .collect();
```

## Tutorial

A four-step hands-on sequence (SciComp-Rust 2026). Each file is **self-contained** —
the model, the likelihoods, and the inference kernel are all defined in the example
file itself, so edit priors and moves freely and rerun. Every file compiles and
converges as shipped; `// EXERCISE:` comments mark specific edits whose effect is
visible in the window.

```sh
cargo run --release --example tutorial_01_ground   # a scene is a probabilistic program
cargo run --release --example tutorial_02_object   # add a cube; block moves vs. drift
cargo run --release --example tutorial_03_color    # a second observation channel (RGB-D)
cargo run --release --example sandbox              # the full cube pipeline, yours to break
```

1. **tutorial_01_ground** — infer camera height/roll from a depth image of a ground
   plane. Likelihood = a `Distribution` impl, model = a `dyngen!` function, inference
   = one `regen_mh` move.
2. **tutorial_02_object** — a cube with unknown position/size. Custom drift proposal
   (commented out — enable it and watch convergence improve).
3. **tutorial_03_color** — the cube becomes a Rubik's cube; depth + color observed
   jointly. Channel-ablation exercises.
4. **sandbox** — the real-world cube pipeline (orbital camera, metric scale,
   illuminant tint) with extension ideas, plus a commented RealSense swap: the same
   model and kernel drive live camera frames by changing one harness call.

All windows share the harness keys: **Space** pause, **R** fresh scene, **S**
snapshot, **ESC** quit.

## Running the tests

```sh
cargo test --release --test derender
```

Renders MP4s to `out/`. Resolution and clipping planes are runtime-configurable (default
64×64, tuned for the synthetic models):

```sh
RES=128 cargo test --release --test derender test_derender_mug
```

## The cube pipeline environment

Everything touching `cube_rgbd_model` — the rubiks test, `derender_window rubiks`, dataset
generation, `synth_cnn`, and the live demos — must run in the **same** environment, because
the CNN bakes the FOV and the depth encoding into its weights. The cube-only binaries
(`gen_dataset`, `live_rgbd_cube`, `synth_cnn`, `live_cnn`) apply these defaults themselves
(`config::apply_cube_pipeline_defaults`); env vars still override them. Multi-model
commands (the rubiks test, `derender_window rubiks`) need them set explicitly:

```sh
RES=128 FOVY_RAD=0.74 NEAR_M=0.01 FAR_M=1.0
```

- `FOVY_RAD=0.74` — ≈42.5°, the D435 color stream's vertical FOV (the live loop calibrates
  the exact value from the camera intrinsics at startup).
- `NEAR_M=0.01` — below the closest ground pixel the orbital camera can ever see (~6cm at
  grazing elevation), so no visible pixel ever hits the near plane.
- `FAR_M=1.0` — keeps depth-likelihood contrast over the 0.25–0.7m working range instead of
  squashing it into the top of a 7.5m window.

A `FOVY_RAD`/`NEAR_M`/`FAR_M` mismatch between dataset generation and inference silently
degrades the CNN — the values are recorded in the dataset's `meta.txt` for auditability.

## Live RealSense demo

`derender/src/bin/live_rgbd_cube.rs` derenders a real Intel RealSense D435 depth+color
stream against `cube_rgbd_model` in real time — tracking a physical cube's pose via depth
(no lighting/material sim-to-real gap) and its face orientation via a flat, no-lighting
color render (viable because real stickers are already flat and saturated). Requires
`librealsense2` and a connected D400-series camera.

```sh
cargo run --release --bin live_rgbd_cube
```

- `RES` — render resolution (default 64).
- `--path-trace` — use the path-traced renderer instead of the flat one (visuals only; the
  Monte Carlo noise corrupts MH acceptance ratios, so inference quality degrades).
- `--d-noise` / `--rgb-noise` — observation noise levels; `--rgb-noise 0.99` ablates the
  color likelihood entirely.

`live_cnn` (in `pose-net/`) is the same loop plus a CNN-guided MH proposal from a trained
PoseNet — see below.

## CNN-guided inference (pose-net)

The generative model doubles as a training-data pipeline: `gen_dataset` samples poses and
renders from `cube_rgbd_model`, `pose-net` trains a small CNN (Candle, CUDA) to regress the
orbit pose, and the estimate drives an independence MH proposal (`pose_guide`) that the
acceptance test keeps honest — zero real-world labels anywhere.

```sh
# generate 50k synthetic training pairs (~13 GB)
cargo run --release --bin gen_dataset -- --n 50000

# train (GPU); --take caps RAM usage
cargo run --release -p pose-net --features cuda --bin train -- --lr 3e-4 --epochs 50 --take 25000

# print or render the CNN architecture with Graphviz
cargo run -p pose-net --bin graphviz -- --output out/pose_net.svg

# synthetic validation: prints CNN vs ground truth, C toggles the guided move live
cargo run --release -p pose-net --bin synth_cnn

# live camera with the CNN proposal
cargo run --release -p pose-net --bin live_cnn
```

Opens a window with observed depth/color (top) and the live inferred hypothesis (bottom).
