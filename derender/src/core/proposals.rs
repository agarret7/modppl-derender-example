use modppl::prelude::*;
use crate::image::*;


dyngen!(
pub fn rgbd_drift(trace: Weak<DynTrace<(f32,f32,bool),(Depths,Colors)>>, mask: Vec<&str>, stdev: f64) {
    let trace = trace.upgrade().unwrap();
    for addr in mask.iter() {
        normal(trace.data.read::<f64>(addr), stdev) %= addr;
    }
});

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
pub fn pose_guide(_trace: Weak<DynTrace<(f32,f32,bool),(Depths,Colors)>>, targets: Vec<(&str, f64, f64)>) {
    for (addr, center, stdev) in targets.iter() {
        normal(*center, *stdev) %= addr;
    }
});

dyngen!(
pub fn gaussian_drift(trace: Weak<DynTrace<(),Colors>>, mask: Vec<&str>, stdev: f64) {
    let trace = trace.upgrade().unwrap();
    for addr in mask.iter() {
        normal(trace.data.read::<f64>(addr), stdev) %= addr;
    }
});

dyngen!(
pub fn noise_drift(trace: Weak<DynTrace<f32,Colors>>, mask: Vec<&str>, stdev: f64) {
    let trace = trace.upgrade().unwrap();
    for addr in mask.iter() {
        normal(trace.data.read::<f64>(addr), stdev) %= addr;
    }
});