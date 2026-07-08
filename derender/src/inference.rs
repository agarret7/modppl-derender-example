use modppl::prelude::*;

/// A composable MCMC sweep over a fixed `model`: an ordered sequence of moves
/// (each a `regen_mh`, `mh`, or any other `Trace -> (Trace, bool)` step) built
/// up with `.then(...)`. Built without a trace -- once assembled, drive it
/// either way:
///   - `.step(trace)` runs one sweep and returns the new trace, for callers
///     that want to own the trace between sweeps (e.g. a live loop's
///     per-frame restart logic);
///   - `.iter(trace)` returns an infinite iterator of successive sweep
///     results seeded at `trace`, for the usual
///     `.take(n).map(...).collect()` pipelines.
pub struct InferenceKernel<'a, M, Args, Data, Ret> {
    model: &'a M,
    moves: Vec<Box<dyn Fn(Trace<Args, Data, Ret>) -> (Trace<Args, Data, Ret>, bool) + 'a>>,
}

impl<'a, M, Args, Data, Ret> InferenceKernel<'a, M, Args, Data, Ret> {
    pub fn new(model: &'a M) -> Self {
        Self {
            model,
            moves: vec![],
        }
    }

    /// appends one move to the sweep, applied in the order added.
    pub fn then<F>(mut self, mv: F) -> Self
    where
        F: Fn(Trace<Args, Data, Ret>) -> (Trace<Args, Data, Ret>, bool) + 'a,
    {
        self.moves.push(Box::new(mv));
        self
    }

    /// runs one full sweep (every move, in order) over `trace` and returns.
    pub fn step(&self, trace: Trace<Args, Data, Ret>) -> Trace<Args, Data, Ret> {
        let mut trace = trace;
        for mv in &self.moves {
            trace = mv(trace).0;
        }
        trace
    }
}

impl<'a, M, Args, Data, Ret> InferenceKernel<'a, M, Args, Data, Ret>
where
    M: GenFn<Args, Data, Ret>,
    Args: Clone + 'static,
    Data: Clone + 'static,
    Ret: Clone + 'static,
{
    /// appends a `regen_mh` move over `pass`, against the kernel's model.
    pub fn regen_mh(self, pass: &'a AddrMap) -> Self {
        let model = self.model;
        self.then(move |t| modppl::regenerative_metropolis_hastings(model, t, pass))
    }

    /// appends an `mh` move using `proposal` with the given `proposal_args`,
    /// against the kernel's model.
    pub fn mh<ProposalArgs: Clone + 'a>(
        self,
        proposal: &'a impl GenFn<(Weak<Trace<Args, Data, Ret>>, ProposalArgs), Data, ()>,
        proposal_args: ProposalArgs,
    ) -> Self {
        let model = self.model;
        self.then(move |t| modppl::metropolis_hastings(model, t, proposal, proposal_args.clone()))
    }
}

impl<'a, M, Args: Clone, Data: Clone, Ret: Clone> InferenceKernel<'a, M, Args, Data, Ret> {
    /// an infinite iterator of successive sweep results, seeded with
    /// `trace`, meant to be bounded with `.take(n)`.
    pub fn iter(&self, trace: Trace<Args, Data, Ret>) -> impl Iterator<Item = Trace<Args, Data, Ret>> + '_ {
        std::iter::successors(Some(self.step(trace)), move |t| Some(self.step(t.clone())))
    }
}
