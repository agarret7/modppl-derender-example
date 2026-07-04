use anyhow::Result;

pub const N_AZ_OUTPUTS: usize = 2;
pub const N_ER_OUTPUTS: usize = 2;

/// (in_channels, out_channels) per stride-2 layer; each halves the spatial size.
pub const LAYERS: [(usize, usize); 5] = [(4, 32), (32, 64), (64, 128), (128, 128), (128, 256)];

#[derive(Clone, Copy, Debug)]
pub struct ConvLayerInfo {
    pub index: usize,
    pub cin: usize,
    pub cout: usize,
    pub stride: usize,
    pub in_h: usize,
    pub in_w: usize,
    pub out_h: usize,
    pub out_w: usize,
}

pub fn conv_arch(h: usize, w: usize) -> Result<(Vec<ConvLayerInfo>, usize)> {
    let final_cells = match (h, w) {
        (64, 64) => 4 * 4,
        (128, 128) => 4 * 4,
        _ => anyhow::bail!("unsupported input size {h}x{w} (need 64 or 128 square)"),
    };
    let last_stride_1 = h == 64;

    let mut hh = h;
    let mut ww = w;
    let mut layers = Vec::with_capacity(LAYERS.len());
    for (index, &(cin, cout)) in LAYERS.iter().enumerate() {
        let stride = if last_stride_1 && index == LAYERS.len() - 1 { 1 } else { 2 };
        // All PoseNet convolutions use kernel=3, padding=1, dilation=1.
        let out_h = (hh + 2 - 3) / stride + 1;
        let out_w = (ww + 2 - 3) / stride + 1;
        layers.push(ConvLayerInfo { index, cin, cout, stride, in_h: hh, in_w: ww, out_h, out_w });
        hh = out_h;
        ww = out_w;
    }

    Ok((layers, final_cells))
}

/// Graphviz DOT description of the PoseNet architecture for `h`x`w` inputs.
///
/// Print it directly with the `graphviz` binary, or pipe it to Graphviz:
/// `cargo run -p pose-net --bin graphviz -- --h 128 --w 128 | dot -Tsvg > out/pose_net.svg`.
pub fn graphviz_dot(h: usize, w: usize) -> Result<String> {
    let (layers, final_cells) = conv_arch(h, w)?;
    let fc_in = 256 * final_cells;

    let mut dot = String::new();
    dot.push_str("digraph PoseNet {\n");
    dot.push_str("  graph [rankdir=TB, fontname=\"Helvetica\", labelloc=t, label=\"PoseNet CNN\", nodesep=0.35, ranksep=0.45, margin=0.08];\n");
    dot.push_str("  node [shape=record, style=\"rounded,filled\", fillcolor=\"#f8fbff\", color=\"#4b6478\", fontname=\"Helvetica\", fontsize=11, margin=\"0.08,0.05\"];\n");
    dot.push_str("  edge [color=\"#4b6478\", arrowsize=0.7, fontname=\"Helvetica\"];\n\n");
    dot.push_str(&format!(
        "  input [label=\"{{input|1 x 4 x {h} x {w}|depth, B, G, R}}\", fillcolor=\"#edf7ed\"];\n"
    ));

    for layer in &layers {
        dot.push_str(&format!(
            "  conv{index} [label=\"{{conv{index} + ReLU|3x3, stride {stride}, pad 1|{cin} x {in_h} x {in_w} to {cout} x {out_h} x {out_w}}}\"];\n",
            index = layer.index,
            stride = layer.stride,
            cin = layer.cin,
            in_h = layer.in_h,
            in_w = layer.in_w,
            cout = layer.cout,
            out_h = layer.out_h,
            out_w = layer.out_w,
        ));
    }

    dot.push_str(&format!(
        "  flatten [label=\"{{flatten|256 x 4 x 4 to {fc_in}}}\", fillcolor=\"#fff8e8\"];\n"
    ));
    dot.push_str(&format!(
        "  fc [label=\"{{fc + ReLU|Linear|{fc_in} to 256}}\", fillcolor=\"#fff8e8\"];\n"
    ));
    dot.push_str(&format!(
        "  head_az [label=\"{{head_az|Linear|256 to {N_AZ_OUTPUTS}|sin az, cos az}}\", fillcolor=\"#f4efff\"];\n"
    ));
    dot.push_str(&format!(
        "  head_er [label=\"{{head_er|Linear|256 to {N_ER_OUTPUTS}|sin elevation, radius}}\", fillcolor=\"#f4efff\"];\n\n"
    ));

    dot.push_str("  input -> conv0;\n");
    for i in 0..LAYERS.len() - 1 {
        dot.push_str(&format!("  conv{i} -> conv{};\n", i + 1));
    }
    dot.push_str("  conv4 -> flatten -> fc;\n");
    dot.push_str("  fc -> head_az;\n");
    dot.push_str("  fc -> head_er;\n");
    dot.push_str("  { rank=same; head_az; head_er; }\n");
    dot.push_str("}\n");

    Ok(dot)
}
