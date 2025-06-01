use std::ops::{AddAssign, Deref, DerefMut, MulAssign};

use tolerance::{Tolerance, Tolerances, compute_error};

use crate::state::OdeState;

#[derive(Clone, Copy, Debug)]
pub struct StateArray<const N: usize>([f64; N]);

impl<const N: usize> StateArray<N> {
    pub fn new(array: [f64; N]) -> Self {
        Self(array)
    }
}

impl<const N: usize> Default for StateArray<N> {
    fn default() -> Self {
        Self([0.0; N])
    }
}

impl<const N: usize> AddAssign<&Self> for StateArray<N> {
    fn add_assign(&mut self, rhs: &Self) {
        for i in 0..N {
            self.0[i] += rhs.0[i];
        }
    }
}

impl<const N: usize> MulAssign<f64> for StateArray<N> {
    fn mul_assign(&mut self, rhs: f64) {
        for i in 0..N {
            self.0[i] *= rhs;
        }
    }
}

impl<const N: usize> OdeState for StateArray<N> {
    type Tolerance = StateArrayTolerances<N>;
    type Value = StateArrayValue<N>;
}

impl<const N: usize> Deref for StateArray<N> {
    type Target = [f64; N];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize> DerefMut for StateArray<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub struct StateArrayValue<const N: usize>([f64; N]);

pub struct StateArrayTolerances<const N: usize>([Option<Tolerances>; N]);
impl<const N: usize> Tolerance for StateArrayTolerances<N> {
    type State = StateArray<N>;
    fn compute_error(
        &self,
        x: &StateArray<N>,
        x_prev: &StateArray<N>,
        x_tilde: &StateArray<N>,
        rel_tol: f64,
        abs_tol: f64,
    ) -> f64 {
        if N == 0 {
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
        (sum_squared_errors / N as f64).sqrt()
    }
}

impl<const N: usize> Default for StateArrayTolerances<N> {
    fn default() -> Self {
        Self([None; N])
    }
}
