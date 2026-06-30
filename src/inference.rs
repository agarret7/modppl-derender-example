use modppl::prelude::*;

/// A composable MCMC sweep over a fixed `model`: an ordered sequence of moves
/// (each a `regen_mh`, `mh`, or any other `Trace -> (Trace, bool)` step) built
/// up with `.then(...)`. Iterating it runs one full sweep per `.next()` call,
/// threading the trace through automatically. This replaces the usual
/// `let (new_trace, _) = regen_mh(&model, ...); trace = new_trace;` chain with
/// a single declarative pipeline you can drive with ordinary iterator
/// combinators (`.take(n)`, `.map(...)`, `.collect()`).
///
/// Never returns `None`: it's an infinite iterator over successive sweep
/// results, meant to be bounded with `.take(n)`.
pub struct Kernel<'a, M, Args, Data, Ret> {
    model: &'a M,
    trace: Option<Trace<Args,Data,Ret>>,
    moves: Vec<Box<dyn Fn(Trace<Args,Data,Ret>) -> (Trace<Args,Data,Ret>, bool) + 'a>>,
}

impl<'a, M, Args, Data, Ret> Kernel<'a, M, Args, Data, Ret> {
    pub fn new(model: &'a M, trace: Trace<Args,Data,Ret>) -> Self {
        Self { model, trace: Some(trace), moves: vec![] }
    }

    /// appends one move to the sweep, applied in the order added.
    pub fn then<F>(mut self, mv: F) -> Self
    where
        F: Fn(Trace<Args,Data,Ret>) -> (Trace<Args,Data,Ret>, bool) + 'a,
    {
        self.moves.push(Box::new(mv));
        self
    }
}

impl<'a, M, Args, Data, Ret> Kernel<'a, M, Args, Data, Ret>
where
    M: GenFn<Args,Data,Ret>,
    Args: Clone + 'static,
    Data: Clone + 'static,
    Ret: Clone + 'static,
{
    /// appends a `regen_mh` move over `mask`, against the kernel's model.
    pub fn regen_mh(self, mask: &'a AddrMap) -> Self {
        let model = self.model;
        self.then(move |t| modppl::regenerative_metropolis_hastings(model, t, mask))
    }

    /// appends an `mh` move using `proposal` with the given `proposal_args`,
    /// against the kernel's model.
    pub fn mh<ProposalArgs: Clone + 'a>(
        self,
        proposal: &'a impl GenFn<(Weak<Trace<Args,Data,Ret>>,ProposalArgs),Data,()>,
        proposal_args: ProposalArgs,
    ) -> Self {
        let model = self.model;
        self.then(move |t| modppl::metropolis_hastings(model, t, proposal, proposal_args.clone()))
    }
}

impl<'a, M, Args: Clone, Data: Clone, Ret: Clone> Iterator for Kernel<'a, M, Args, Data, Ret> {
    type Item = Trace<Args,Data,Ret>;

    fn next(&mut self) -> Option<Self::Item> {
        let mut trace = self.trace.take()?;
        for mv in &self.moves {
            trace = mv(trace).0;
        }
        self.trace = Some(trace.clone());
        Some(trace)
    }
}
