use std::ops::{AddAssign, MulAssign};

pub mod rk4;

pub struct SimulationConfig {
    pub name: String,
    pub start: f64,
    pub stop: f64,
    pub dt: f64,
}

#[derive(Debug, Clone)]
pub struct SimStateVector(pub Vec<f64>);
impl MulAssign<f64> for SimStateVector {
    fn mul_assign(&mut self, rhs: f64) {
        self.0.iter_mut().for_each(|state| *state *= rhs);
    }
}
impl AddAssign<&Self> for SimStateVector {
    fn add_assign(&mut self, rhs: &Self) {
        self.0
            .iter_mut()
            .zip(rhs.0.iter()) // Use `iter()` to iterate immutably over `rhs`
            .for_each(|(left, right)| *left += right);
    }
}

#[derive(Debug, Clone)]
pub struct SimStates(pub Vec<SimStateVector>);

impl MulAssign<f64> for SimStates {
    fn mul_assign(&mut self, rhs: f64) {
        self.0.iter_mut().for_each(|state| *state *= rhs);
    }
}
impl AddAssign<&Self> for SimStates {
    fn add_assign(&mut self, rhs: &Self) {
        self.0
            .iter_mut()
            .zip(rhs.0.iter())
            .for_each(|(left, right)| *left += right);
    }
}
