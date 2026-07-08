pub type Depth = f32; // in [0.0, 1.0]
pub type Color = [f32; 3]; // in [0.0, 1.0] (BGR)

pub type Depths = Vec<Depth>;
pub type Colors = Vec<Color>;

/// Builds a `Color` from red/green/blue components in the order you'd
/// actually expect. `Color` is stored BGR internally (matching the BMP-native
/// byte layout the rest of the renderer assumes -- see `serialization.rs`),
/// so a bare array literal like `[0.8, 0.2, 0.2]` is quietly *blue*, not red.
/// Prefer this constructor over writing `Color` literals directly.
pub fn rgb(r: f32, g: f32, b: f32) -> Color {
    [b, g, r]
}
