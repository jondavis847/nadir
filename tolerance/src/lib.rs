#[derive(Debug, Clone, Copy)]
pub struct Tolerances {
    pub abs_tol: f64,
    pub rel_tol: f64,
}

impl Default for Tolerances {
    fn default() -> Self {
        Self {
            abs_tol: 1e-6,
            rel_tol: 1e-3,
        }
    }
}
impl Tolerances {
    pub fn new(rel_tol: f64, abs_tol: f64) -> Self {
        Self { rel_tol, abs_tol }
    }
    pub fn compute_error(&self, x: f64, x_prev: f64, x_tilde: f64) -> f64 {
        compute_error(x, x_prev, x_tilde, self.rel_tol, self.abs_tol)
    }
}

pub trait Tolerance: Default {
    type State;
    fn compute_error(
        &self,
        x: &Self::State,
        x_prev: &Self::State,
        x_tilde: &Self::State,
        rel_tol: f64,
        abs_tol: f64,
    ) -> f64;
}

/// Computes the error for determining adaptive step size
/// When looking at b coefficients, there's usually a b and a bhat, btilde is the different between b and bhat
/// Be away that x is the currently calcualted state, x_prev is the initial state of the step, x_tilde is
/// calculated from dt * sum(b_tilde[i] * k[i]), NOT y_hat - y
pub fn compute_error(x: f64, x_prev: f64, x_tilde: f64, rel_tol: f64, abs_tol: f64) -> f64 {
    x_tilde / (abs_tol + x.abs().max(x_prev.abs()).max(1e-14) * rel_tol)
}

pub fn compute_component_error(
    opt_tol: &Option<Tolerances>,
    x: f64,
    x_prev: f64,
    x_tilde: f64,
    rel_tol: f64,
    abs_tol: f64,
) -> f64 {
    match opt_tol {
        Some(tol) => tol.compute_error(x, x_prev, x_tilde),
        None => compute_error(x, x_prev, x_tilde, rel_tol, abs_tol),
    }
}
