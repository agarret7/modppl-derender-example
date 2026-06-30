use modppl::prelude::*;
use modppl_derender::{
    core::*,
    config::{W, H},
    realsense::*,
};
use minifb::{Window, WindowOptions, Key, Scale};

const NOISE: f32 = 0.1;
const SWEEPS_PER_FRAME: usize = 5;

/// `lm` is the model's normalized inverse-depth convention: 1.0 at NEAR, 0.0 at/beyond FAR.
fn lm_to_gray(v: f32) -> u32 {
    let g = (v.clamp(0.0, 1.0) * 255.0) as u32;
    (g << 16) | (g << 8) | g
}

fn main() -> anyhow::Result<()> {
    let mut pipeline = open_depth_pipeline()?;
    println!("RealSense pipeline open, capturing first frame...");

    // initialize the trace from the first observed frame
    let frame = read_depth_frame(&mut pipeline)?;
    let observed = depths_from_frame(&frame);
    let mut constraints = DynTrie::new();
    constraints.observe("observation", Arc::new(observed));
    let mut trace = cube_depth_model.generate(NOISE, constraints).0;

    let mut cam_mask = AddrMap::new();
    cam_mask.visit("cam_y");
    cam_mask.visit("cam_yaw");

    let mut cube_mask = AddrMap::new();
    cube_mask.visit("cube_u");
    cube_mask.visit("cube_v");
    cube_mask.visit("cube_size");

    let mut window = Window::new(
        "live cube derender  |  observed : hypothesis  |  ESC to quit",
        2*W(), H(),
        WindowOptions { scale: Scale::X8, ..WindowOptions::default() }
    )?;

    let mut buffer = vec![0u32; 2*W()*H()];

    while window.is_open() && !window.is_key_down(Key::Escape) {
        // capture a new frame and swap it in, warm-starting from the current trace
        let frame = read_depth_frame(&mut pipeline)?;
        let observed = depths_from_frame(&frame);

        let mut constraints = DynTrie::new();
        constraints.observe("observation", Arc::new(observed.clone()));
        let (new_trace, _discard, _weight) = cube_depth_model.update(trace, NOISE, ArgDiff::NoChange, constraints);
        trace = new_trace;

        // a few MH sweeps to adapt the latents to the new observation
        for _ in 0..SWEEPS_PER_FRAME {
            let (t, _) = regen_mh(&cube_depth_model, trace, &cam_mask);
            trace = t;
            let (t, _) = regen_mh(&cube_depth_model, trace, &cube_mask);
            trace = t;
            let (t, _) = mh(&cube_depth_model, trace, &depth_drift, (vec!["cube_u", "cube_v", "cube_size"], 0.05));
            trace = t;
        }

        let hypothesis = trace.retv.clone().unwrap();

        for y in 0..H() {
            for x in 0..W() {
                buffer[y*2*W() + x] = lm_to_gray(observed[y*W() + x]);
                buffer[y*2*W() + W() + x] = lm_to_gray(hypothesis[y*W() + x]);
            }
        }

        window.update_with_buffer(&buffer, 2*W(), H())?;
    }

    Ok(())
}
