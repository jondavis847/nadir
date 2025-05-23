use std::ops::{AddAssign, Deref, DerefMut, MulAssign};

use tolerance::Tolerance;

use crate::Integrable;

use super::state_vector::{StateVector, StateVectorTolerances};

#[derive(Clone, Debug, Default)]
pub struct StateVectors(Vec<StateVector>);

impl StateVectors {
    pub fn new() -> Self {
        Self(Vec::new())
    }
}

impl AddAssign<&Self> for StateVectors {
    fn add_assign(&mut self, rhs: &Self) {
        if self.0.len() != rhs.0.len() {
            panic!("state vectors do not have same length")
        }
        for i in 0..self.len() {
            self.0[i] += &rhs.0[i];
        }
    }
}

impl MulAssign<f64> for StateVectors {
    fn mul_assign(&mut self, rhs: f64) {
        for i in 0..self.len() {
            self.0[i] *= rhs;
        }
    }
}

impl Integrable for StateVectors {
    type Derivative = Self;
    type Tolerance = StateVectorsTolerances;
}

impl Deref for StateVectors {
    type Target = Vec<StateVector>;

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl DerefMut for StateVectors {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

#[derive(Default)]
pub struct StateVectorsTolerances(Vec<StateVectorTolerances>);
impl Tolerance for StateVectorsTolerances {
    type State = StateVectors;
    fn compute_error(
        &self,
        x: &StateVectors,
        x_prev: &StateVectors,
        x_tilde: &StateVectors,
        rel_tol: f64,
        abs_tol: f64,
    ) -> f64 {
        if x.len() == 0 {
            return 0.0;
        }
        let mut sum_squared_errors = 0.0;

        // Compute the squared error for each component
        for (i, tol) in self.0.iter().enumerate() {
            let component_error =
                tol.compute_error(&x.0[i], &x_prev.0[i], &x_tilde.0[i], rel_tol, abs_tol);
            sum_squared_errors += component_error * component_error;
        }

        // Return the root mean square error
        (sum_squared_errors / x.len() as f64).sqrt()
    }
}
