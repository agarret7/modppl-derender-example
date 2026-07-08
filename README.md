# modppl-derender

Inverse graphics in [ModPPL](https://crates.io/crates/modppl): combine a rendering-based
generative scene model with user-space MCMC inference to recover structured 3D scene
hypotheses directly from images. Each scene is a probabilistic program (a prior over camera
pose, object pose/size/color, and lighting) paired with a ray-traced renderer used as the
observation likelihood. Inference is plain Metropolis-Hastings over the program's own random
choices, with custom proposals where the default ones aren't enough.

## Setup

Install [Rust](https://rust-lang.org/tools/install) (stable) and Git, then:

```sh
git clone https://github.com/agarret7/modppl-derender
cd modppl-derender

cargo build --release --examples
RES=128 cargo run --release --example sandbox -- ball
```

If a two-panel animation appears, you're ready. The live examples open a window
(via [`minifb`](https://crates.io/crates/minifb)), which needs a C compiler and
windowing headers already present on most systems. if the build fails looking
for them:

- **Linux**: `sudo apt install build-essential libxkbcommon-dev libwayland-dev`
  (or your distro's equivalent; X11 or Wayland packages depending on your desktop)
- **macOS**: `xcode-select --install`
- **Windows**: MSVC toolchain installed alongside Rust covers it

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
| [![ball](out/thumbs/ball.png)](out/ball.mp4)<br>**Ball, from a real photo** — derendering an actual `.bmp` photograph, not a synthetic render | [![live rgbd](out/thumbs/live_rgbd.png)](out/live_rgbd.mp4)<br>**Live RealSense demo** — real depth+color camera input (top) vs. the live-inferred cube hypothesis (bottom), via `live_rgbd_cube` |

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

Each lives in its own `examples/scenes/*.rs` file (`cube_rgbd_model` is the exception,
importable from [`derender/src/core/cube_model.rs`](derender/src/core/cube_model.rs) since
it has real cross-crate consumers), written with the `dyngen!` macro. Inference sweeps are
built with the `InferenceKernel` combinator
([`derender/src/inference.rs`](derender/src/inference.rs)):

```rust
let kernel = InferenceKernel::new(&ball_model)
    .regen_mh(&cam_pass)
    .regen_mh(&env_pass)
    .mh(&gaussian_drift, (table_color_pass.clone(), 0.1))
    .regen_mh(&ball_pass)
    .mh(&gaussian_drift, (ball_pass.clone(), 0.1))
    .regen_mh(&ball_color_pass);

let renders: Vec<Colors> = kernel
    .iter(trace)
    .take(NUM_ITERS)
    .map(|t| t.retv.clone().unwrap())
    .collect();
```

## Tutorial

A five-step hands-on sequence (SciComp-Rust 2026 workshop). Each file is
**self-contained** — the model, the likelihoods, and the inference kernel are all
defined in the example file itself, so edit priors and moves freely and rerun. Every
file compiles and converges as shipped; `// EXERCISE:` comments mark specific edits
whose effect is visible in the window.

**Setup:**

```sh
git clone <this repo>
cd modppl-derender
cargo build --release --examples
```

**Run in order:**

```sh
cargo run --release --example tutorial_01_intro    # what is a probabilistic program?
cargo run --release --example tutorial_02_ground   # a scene is a probabilistic program
cargo run --release --example tutorial_03_object   # add a cube; block moves vs. drift
cargo run --release --example tutorial_04_color    # a second observation channel (RGB-D)
cargo run --release --example sandbox -- rubiks    # the full cube pipeline, yours to break
```

1. **tutorial_01_intro** — probabilistic programming in five minutes: write a model
   with `dyngen!`, sample it, condition it on observed values, and watch MH inference
   recover a hidden number.
2. **tutorial_02_ground** — the leap to inverse graphics: put a ray tracer *inside* the model, so "explain this depth image" becomes ordinary Bayesian inference over camera height and roll. One `regen_mh` move is enough.
3. **tutorial_03_object** — add a cube with unknown position and size, and observe inference tradeoffs: prior-resampling moves find the right region, a drift proposal (commented out — turn it on) refines it.
4. **tutorial_04_color** — observe color alongside depth. One extra render call and one new `%=` sample line transform the generative function's observation from
   `Depths ≅ Vec<f32>` to
   `RGB-D ≅ (Depths, Colors)` where `Colors ≅ Vec<[f32; 3]>`.
   Ablate either channel and see what each one does (and doesn't) constrain.
5. **sandbox** — one entrypoint to every demo scene: `-- ground`, `-- sphere`, `-- ball`,
   `-- mug`, `-- cone_sphere`, or `-- rubiks`. Each lives in its own file under
   `examples/scenes/` — open it, edit the model or kernel, rerun. The `rubiks` scene
   is the full real-world cube pipeline at metric scale, with extension ideas
   throughout; if you have a RealSense camera, try real-time inference.

All windows share a few controls: **Space** pause, **R** fresh scene, **S** snapshot, **ESC** quit.

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

Everything touching `cube_rgbd_model` (the rubiks test and scene, dataset generation,
`synth_cnn`, and the live demos) shares one metric-scale environment via
`config::apply_cube_pipeline_defaults()` (`RES=128`, `FOVY_RAD=0.74`, `NEAR_M=0.01`,
`FAR_M=1.0`). env vars still override it. A mismatch between dataset generation and
inference silently degrades the CNN, since it bakes the FOV and depth encoding into its
weights; see `config.rs` for what each value means and why.

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
orbit pose, and the estimate is called inside an MH proposal (`pose_guide`).

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
