pub mod rk4;
use super::MultibodySystem;

pub trait Integrable<T> 

pub enum SolverMethod {
    ClassicalRk4,
    DormandPrince45,
    Tsitouras45,
}

pub struct Solver {
    current_time: f64,
    last_time: f64,
    end_time: f64,
    delta_time: f64,
    method: SolverMethod,
}

impl Solver {
    pub fn new(tspan: (f64, f64), dt: f64) -> Self {
        Self {
            current_time: tspan.0,
            last_time: tspan.0,
            end_time: tspan.1,
            delta_time: dt,
            method: SolverMethod::ClassicalRk4,
        }
    }
}

pub trait SolverTrait {
    fn solve(&mut self, sys: &mut MultibodySystem);
}
