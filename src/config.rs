use std::f32::consts::PI;


/* constants */

pub const H   : usize = 128;
pub const W   : usize = 128;
pub const AREA: usize = H*W;
pub const VIEWPORT: [f32; 4] = [0.0, 0.0, W as f32, H as f32];

pub const FOVY: f32 = PI/2.0;
pub const NEAR: f32 = 0.2;
pub const FAR : f32 = 7.5;