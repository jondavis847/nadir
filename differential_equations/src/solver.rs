use super::{Integrable, OdeFunction};

pub mod rk4;

use rk4::solve_fixed_rk4;

pub enum SolverMethod {
    Rk4Classical,
    //DormandPrince45,
    //Tsitouras54,
}

pub struct Solver<F, P, T>
where
    F: OdeFunction<P, T>,
    T: Integrable,
{
    pub func: F,
    pub x0: T,
    pub parameters: Option<P>,
    pub tstart: f64,
    pub tstop: f64,
    pub dt: f64,
    pub solver: SolverMethod,
    pub callbacks: Vec<Box<dyn FnMut(&mut T, &mut Option<P>, &mut f64)>>,
}

impl<F, P, T> Solver<F, P, T>
where
    F: OdeFunction<P, T>,
    T: Integrable,
{
    // Method to run the solver
    pub fn solve(&mut self) -> (Vec<f64>, Vec<T>) {
        match self.solver {
            SolverMethod::Rk4Classical => solve_fixed_rk4(self),
        }
    }
}
