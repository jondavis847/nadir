use std::ops::{Add, Div, Mul};
use super::Integrable;

pub mod rk4;
use rk4::Rk4;
pub enum SolverMethod {
    ClassicalRk4,
    DormandPrince45,
    Tsitouras45,
}

pub struct Solver {}

pub trait SolverTrait {
    fn solve_fixed<T, F>(&mut self, x0: T, func: F, tspan: (f64, f64), dt: f64) -> (Vec<f64>,Vec<T>)
    where
        T: Integrable,
        F: Fn(T, f64) -> T; // (state, dt)
}
