use modppl::prelude::*;
use crate::image::*;


dyngen!(
pub fn rgbd_drift(trace: Weak<DynTrace<(f32,f32),(Depths,Colors)>>, mask: Vec<&str>, stdev: f64) {
    let trace = trace.upgrade().unwrap();
    for addr in mask.iter() {
        normal(trace.data.read::<f64>(addr), stdev) %= addr;
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