use crate::image::*;
use modppl::prelude::*;

/// Builds an `AddrMap` that visits exactly the given addresses -- shorthand
/// for the usual `let mut m = AddrMap::new(); m.visit(...); ...` when a
/// drift proposal's target set is just a flat list, e.g.
/// `mh(&noise_drift, (pass_of(&["cone/height", "cone/radius"]), 0.05))`.
pub fn pass_of(addrs: &[&str]) -> AddrMap {
    let mut m = AddrMap::new();
    for addr in addrs {
        m.visit(addr);
    }
    m
}

/// Flattens an `AddrMap`'s visited addresses back into "/"-joined strings,
/// mirroring how `AddrMap::visit` parses them going in. Each recursive call
/// only needs to know its own children's local keys -- nested paths are
/// rebuilt on the way back up, not threaded down as an accumulator.
pub fn addrs_of(pass: &AddrMap) -> Vec<String> {
    let mut out = vec![];
    for (k, sub) in pass.iter() {
        if sub.is_leaf() {
            out.push(k.clone());
        } else {
            for addr in addrs_of(sub) {
                out.push(format!("{k}/{addr}"));
            }
        }
    }
    out
}

dyngen!(
    pub fn rgbd_drift(
        trace: Weak<DynTrace<(f32, f32, bool), (Depths, Colors)>>,
        pass: AddrMap,
        stdev: f32,
    ) {
        let trace = trace.upgrade().unwrap();
        for addr in &addrs_of(&pass) {
            normal(trace.data.read::<f32>(addr), stdev) %= addr.as_str();
        }
    }
);

dyngen!(
    /// Independence proposal guided by an external pose estimate (e.g. a trained
    /// CNN): samples each address around a given absolute center, ignoring the
    /// current trace value. Unlike a drift it can rescue a lost chain in one
    /// accepted move; unlike regen_mh it lands near the right answer instead of
    /// anywhere in the prior. The MH acceptance test corrects for estimator error.
    ///
    /// `targets`: (address, center, stdev) triples. Out-of-support proposals
    /// (e.g. azimuth outside [0, 2π)) score -inf under the model prior and are
    /// simply rejected -- same behavior as the existing drift moves near bounds.
    pub fn pose_guide(
        _trace: Weak<DynTrace<(f32, f32, bool), (Depths, Colors)>>,
        targets: Vec<(&str, f32, f32)>,
    ) {
        for (addr, center, stdev) in targets.iter() {
            normal(*center, *stdev) %= addr;
        }
    }
);

dyngen!(
    pub fn gaussian_drift(trace: Weak<DynTrace<(), Colors>>, pass: AddrMap, stdev: f32) {
        let trace = trace.upgrade().unwrap();
        for addr in &addrs_of(&pass) {
            normal(trace.data.read::<f32>(addr), stdev) %= addr.as_str();
        }
    }
);

dyngen!(
    pub fn noise_drift(trace: Weak<DynTrace<f32, Colors>>, pass: AddrMap, stdev: f32) {
        let trace = trace.upgrade().unwrap();
        for addr in &addrs_of(&pass) {
            normal(trace.data.read::<f32>(addr), stdev) %= addr.as_str();
        }
    }
);
