//! Tutorial 01: the whole pattern in 5 minutes, no rendering.
//!
//! Guess a hidden number from noisy measurements of it: sample unknowns from
//! priors (`dyngen!`, `%=`), condition on known values (`.generate(args,
//! constraints)`), then run a `InferenceKernel` of MCMC moves to recover the rest.
//! tutorial_02 is this exact pattern with a camera and a depth image standing
//! in for "hidden" and "measurements".
//!
//! Run it (plain terminal output, no window):
//!   cargo run --release --example tutorial_01_intro

use modppl::prelude::*;
use modppl_derender::inference::InferenceKernel;

dyngen!(
    // dyngen! defines _two_ primitive operations: `%=` and `/=`
    // both are of the mathematical form "var ~ random_function(args)"

    // `<dist: Distribution>(params) %= <addr: &str>;`
    // create a random `var` at `addr` with primitive distribution `dist`.

    // `<model: DynGenFn>(args) /= <addr: &str>;
    // create a sub-trace `var` at `addr` with dynamic generaive function `model`.

    fn guess_the_number_model(noise: f32) -> f32 {
        // parens not needed, just included to highlight
        let hidden = (uniform(0.0, 100.0) %= "hidden");
        // i.e. hidden ~ uniform(0.0, 100.0)

        normal(hidden, noise) %= "m0";
        normal(hidden, noise) %= "m1";
        normal(hidden, noise) %= "m2";
        normal(hidden, noise) %= "m3";
        normal(hidden, noise) %= "m4";
        // i.e. m[i] ~ normal(hidden, noise), i in 0..5

        // equivalently in real Rust

        // for i in 0..5 {
        //     normal(hidden, noise) %= &format!("m{i}");
        // }

        hidden
    }
);

// ─── inference ───────────────────────────────────────────────────────────────

fn main() {
    let mut constraints = DynTrie::new();

    // let's make some synthetic measurements
    constraints.observe("m0", Arc::new(40.0_f32));
    constraints.observe("m1", Arc::new(41.0_f32));
    constraints.observe("m2", Arc::new(42.0_f32));
    constraints.observe("m3", Arc::new(43.0_f32));
    constraints.observe("m4", Arc::new(44.0_f32));

    let noise_arg = 5.0;  // EXERCISE: decrease to 0.05;
    let (trace, weight) = guess_the_number_model.generate(noise_arg, constraints);

    // read out the starting guess
    // run a few times, see how it changes
    let h = trace.data.read::<f32>("hidden");
    println!("starting guess = {h}");
    println!("log[p(m0|hidden={h}) \n  * p(m1|hidden={h}) \n  * ... \n  * p(m4|hidden={h})] = {weight}\n");

    // a pass names which addresses are regenerated in **Metropolis-Hastings**.
    let mut hidden_pass = AddrMap::new();
    hidden_pass.visit("hidden");

    // This is a custom Metropolis-Hastings InferenceKernel
    let kernel = InferenceKernel::new(&guess_the_number_model)
        .regen_mh(&hidden_pass);  // regenerate the hidden variable

    let iters = 20;  // EXERCISE: try increasing to around ~20k
    let start = std::time::Instant::now();

    // `InferenceKernel` can return an `Iterator`!
    // First, initialize with the starting guess
    let kernel_it = kernel.iter(trace);

    // Run inference for `iters`.
    // Each step, collect the return value of the trace.
    let guesses: Vec<f32> = kernel_it
        .take(iters)
        .map(|t| t.retv.unwrap())
        .collect();
    let elapsed = start.elapsed();

    // EXERCISE: uncomment to watch the Markov Chain converge
    // println!("starting guess = {h:.2}");
    // for (i, guess) in guesses.iter().enumerate() {
    //     println!("step {i:2}: guess = {guess:.2}");
    // }
    // println!("\n{iters} iters in {elapsed:.2?}");
}
