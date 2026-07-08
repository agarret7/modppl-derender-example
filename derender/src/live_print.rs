//! Live-redraw trace printer, used by the tutorial series' sandbox loops.

use modppl::prelude::*;
use std::fmt::Debug;
use std::io::Write;

/// Prints `trace` in place: moves the cursor up by however many lines the
/// previous call printed, clears to end of screen, then prints again --
/// so each sandbox-loop frame overwrites the last instead of scrolling.
/// `printed_lines` is the caller's own counter, threaded frame to frame.
pub fn print_trace_live<Args: Debug, Ret: Debug>(trace: &DynTrace<Args, Ret>, printed_lines: &mut usize) {
    let text = modppl::dyntrace_to_string_with_options(
        trace,
        &[],
        DynTracePrintOptions::verbose(false).set_logjp(true),
    );
    if *printed_lines > 0 {
        print!("\x1B[{printed_lines}A\x1B[0J");
    }
    print!("{text}");
    std::io::stdout().flush().unwrap();
    *printed_lines = text.lines().count();
}
