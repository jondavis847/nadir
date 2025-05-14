#[derive(Clone, Copy)]
pub struct Tolerances {
    abs_tol: f64,
    rel_tol: f64,
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
    pub fn compute_error(&self, x0: f64, xf: f64) -> f64 {
        compute_error(x0, xf, self.rel_tol, self.abs_tol)
    }
}

pub trait Tolerance: Default {
    type State;
    fn compute_error(&self, x0: &Self::State, xf: &Self::State, rel_tol: f64, abs_tol: f64) -> f64;
}

pub fn compute_error(x0: f64, xf: f64, rel_tol: f64, abs_tol: f64) -> f64 {
    let abs_diff = (xf - x0).abs();

    // Use max of both values for scaling (MATLAB-compatible)
    let scale = abs_tol + rel_tol * x0.abs().max(xf.abs()).max(1e-10);

    // Return scaled error
    abs_diff / scale
}

pub fn compute_component_error(
    opt_tol: &Option<Tolerances>,
    x0: f64,
    xf: f64,
    rel_tol: f64,
    abs_tol: f64,
) -> f64 {
    match opt_tol {
        Some(tol) => tol.compute_error(x0, xf),
        None => compute_error(x0, xf, rel_tol, abs_tol),
    }
}
