#[derive(Clone, Copy)]
pub struct Tolerances {
    abs_tol: f64,
    rel_tol: f64,
}

impl Default for Tolerances {
    fn default() -> Self {
        Self {
            abs_tol: 1e-3,
            rel_tol: 1e-6,
        }
    }
}
impl Tolerances {
    pub fn new(rel_tol: f64, abs_tol: f64) -> Self {
        Self { rel_tol, abs_tol }
    }
    pub fn check_error(&self, x0: f64, xf: f64) -> bool {
        check_error(x0, xf, self.rel_tol, self.abs_tol)
    }
}

pub trait Tolerance: Default {
    type State;
    fn check_error(&self, x0: &Self::State, xf: &Self::State, rel_tol: f64, abs_tol: f64) -> bool;
}

pub fn check_error(x0: f64, xf: f64, rel_tol: f64, abs_tol: f64) -> bool {
    let abs_diff = (xf - x0).abs();
    let rel_diff = if x0.abs() > 1e-10 {
        abs_diff / x0.abs()
    } else {
        0.0
    };
    abs_diff <= abs_tol || rel_diff <= rel_tol
}
