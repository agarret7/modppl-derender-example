use std::fs;
use std::io::Write;
use std::process::{Command, Stdio};

use anyhow::{Context, Result};
use clap::Parser;

#[path = "../arch.rs"]
mod arch;

#[derive(Parser, Debug)]
#[command(about = "Print or render the PoseNet CNN architecture as Graphviz DOT")]
struct Args {
    /// Input image height used by the trained PoseNet.
    #[arg(long, default_value_t = 128)]
    h: usize,

    /// Input image width used by the trained PoseNet.
    #[arg(long, default_value_t = 128)]
    w: usize,

    /// Optional output path. If omitted, DOT is printed to stdout.
    #[arg(short, long)]
    output: Option<String>,

    /// Graphviz output format when --output is set.
    #[arg(long, default_value = "svg")]
    format: String,
}

fn main() -> Result<()> {
    let args = Args::parse();
    let dot = arch::graphviz_dot(args.h, args.w)?;

    let Some(output) = args.output else {
        print!("{dot}");
        return Ok(());
    };

    if args.format == "dot" {
        fs::write(&output, dot).with_context(|| format!("failed to write {output}"))?;
        return Ok(());
    }

    let mut child = Command::new("dot")
        .arg(format!("-T{}", args.format))
        .stdin(Stdio::piped())
        .stdout(fs::File::create(&output).with_context(|| format!("failed to create {output}"))?)
        .spawn()
        .context("failed to run Graphviz `dot`; install graphviz or use --format dot")?;

    child
        .stdin
        .as_mut()
        .expect("dot stdin is piped")
        .write_all(dot.as_bytes())?;
    let status = child.wait().context("failed to wait for Graphviz `dot`")?;
    anyhow::ensure!(status.success(), "Graphviz `dot` exited with {status}");

    Ok(())
}
