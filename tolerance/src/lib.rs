pub struct Tolerance {
    abs_tol: Option<f64>,
    rel_tol: Option<f64>,
}

pub trait Tolerances {
    fn compute_error(&self, error: &Self) -> f64;
}
