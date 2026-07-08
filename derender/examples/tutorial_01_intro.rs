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
    // dyngen! defines _two_ primitive operations: `%=` and `/=`:

    // `let <var> = <dist: Distribution> %= <addr: &str>;`
    // declare the random `var` at `addr` with primitive distribution `dist`.

    // `let <var> = <model: DynGenFn> /= <addr: &str>;
    // declare the sub-trace `var` at `addr` with dynamic generaive function `model`.
    
    fn guess_the_number_model(noise: f32) -> f32 {
        let hidden = uniform(0.0, 100.0) %= "hidden";

        // measurements
        normal(hidden, noise) %= "m0";
        normal(hidden, noise) %= "m1";
        normal(hidden, noise) %= "m2";
        normal(hidden, noise) %= "m3";
        normal(hidden, noise) %= "m4";

        hidden
    }
);

const MEASUREMENT_ADDRS: [&str; 5] = ["m0", "m1", "m2", "m3", "m4"];

fn main() {
    // `DynTrie` is the `data` type of `DynTrace`.
    // synthesize a ground truth: fix "hidden" and sample the rest (the five
    // measurements) from the model's prior distribution
    let mut synth = DynTrie::new();

    // This _mutates_ `synth` by constraining "hidden" to a value, in this case 42.0_f32.
    synth.observe("hidden", Arc::new(42.0_f32)); // yes, Arc is required.
    let s = modppl::dyntrie_to_string(&synth);
    println!("{s}");

    // The Dynamic Modeling Language DSL
    //
    // A dynamic generative function is a `GenFn` (DynGenFn : GenFn).
    // It is also a probabilistic program that records addressable
    // random vars in a modeling struct, called the DynTrace : Trace
    //
    //  struct DynTrace<A,B> {
    //      args: A,
    //      data: DynTrie,  // = Trie<Arc<dyn Any + Send + Sync>>
    //      retv: Option<B>,
    //      logjp: Real
    //  }

    let noise = 5.0; // try a few different noise levels. What do you observe?
    let gt = guess_the_number_model.generate(noise, synth).0;
    let s = modppl::dyntrace_to_string(&gt);
    println!("{s}");

    // Read the randomly-generated "hidden" number.
    // SAFETY: read may panic if the type is wrong or the address is uninhabited.
    //         try changing the type `f32` -> `f64` or "hidden", it should panic.
    let hidden = gt.data.read::<f32>("hidden");
    println!("true secret: {hidden}");

    // Build constraints from the measurements alone (not the full trace)
    // "hidden" is deliberately left unconstrained, since that's exactly the
    // unknown we're inferring.
    let mut constraints = DynTrie::new();
    println!("measurements:");
    for addr in MEASUREMENT_ADDRS {
        let measurement = gt.data.read::<f32>(addr);
        println!("  {addr} = {:.2}", measurement);
        constraints.observe(addr, Arc::new(measurement)); // yes, Arc is required.
    }

    let (trace, weight) = guess_the_number_model.generate(noise, constraints);
    println!("log[p(m0|hidden) x p(m1|hidden) x ... x p(m4|hidden)] = {weight}\n");

    // a pass names which addresses are regenerated. here, just "hidden".
    // sometimes called a "mask" as it identifies a subset of addresses.
    let mut hidden_pass = AddrMap::new();
    hidden_pass.visit("hidden");

    let kernel = InferenceKernel::new(&guess_the_number_model).regen_mh(&hidden_pass);
    let iters = 20; // try increasing to around ~20k

    let start = std::time::Instant::now();
    let guesses: Vec<f32> = kernel
        .iter(trace)
        .take(iters)
        .map(|t| t.retv.unwrap())
        .collect();
    let elapsed = start.elapsed();

    for (i, guess) in guesses.iter().enumerate() {
        println!("iter {i:2}: guess = {guess:.2}");
    }

    println!("\n{iters} iters in {elapsed:.2?}");
}
