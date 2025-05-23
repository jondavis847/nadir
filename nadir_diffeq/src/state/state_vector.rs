use std::ops::{AddAssign, Deref, DerefMut, MulAssign};

use tolerance::{Tolerance, Tolerances, compute_error};

use crate::Integrable;

#[derive(Clone, Debug, Default)]
pub struct StateVector(Vec<f64>);

impl StateVector {
    pub fn new(vec: Vec<f64>) -> Self {
        Self(vec)
    }
}

impl AddAssign<&Self> for StateVector {
    fn add_assign(&mut self, rhs: &Self) {
        if self.0.len() != rhs.0.len() {
            panic!("state vectors do not have same length")
        }
        for i in 0..self.len() {
            self.0[i] += rhs.0[i];
        }
    }
}

impl MulAssign<f64> for StateVector {
    fn mul_assign(&mut self, rhs: f64) {
        for i in 0..self.len() {
            self.0[i] *= rhs;
        }
    }
}

impl Integrable for StateVector {
    type Derivative = Self;
    type Tolerance = StateVectorTolerances;
}

impl Deref for StateVector {
    type Target = Vec<f64>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for StateVector {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Default)]
pub struct StateVectorTolerances(Vec<Option<Tolerances>>);
impl Tolerance for StateVectorTolerances {
    type State = StateVector;
    fn compute_error(
        &self,
        x: &StateVector,
        x_prev: &StateVector,
        x_tilde: &StateVector,
        rel_tol: f64,
        abs_tol: f64,
    ) -> f64 {
        if x.len() == 0 {
            return 0.0;
        }
        let mut sum_squared_errors = 0.0;

        // Compute the squared error for each component
        for (i, tol) in self.0.iter().enumerate() {
            let component_error = if let Some(tol) = tol {
                tol.compute_error(x.0[i], x_prev.0[i], x_tilde.0[i])
            } else {
                compute_error(x.0[i], x_prev.0[i], x_tilde.0[i], rel_tol, abs_tol)
            };

            sum_squared_errors += component_error * component_error;
        }
        // Return the root mean square error
        (sum_squared_errors / x.len() as f64).sqrt()
    }
}
