use super::Integrable;

pub mod rk4;

use rk4::solve_fixed_rk4;

pub enum SolverMethod {
    Rk4Classical,
    //DormandPrince45,
    //Tsitouras54,
}

pub struct Solver<F, T>
where
    F: Fn(T, f64) -> T,    
    T: Integrable,
{
    pub func: F,
    pub x0: T,
    pub tstart: f64,
    pub tstop: f64,
    pub dt: f64,
    pub solver: SolverMethod,
}

impl<F, T> Solver<F, T>
where
    F: Fn(T, f64) -> T,    
    T: Integrable,
{
    // Method to run the solver
    pub fn solve(&self) -> (Vec<f64>, Vec<T>) {
        match self.solver {
            SolverMethod::Rk4Classical => solve_fixed_rk4(self)
        }        
    }
}