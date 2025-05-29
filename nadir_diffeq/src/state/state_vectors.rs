use crate::Integrable;
use crate::saving::StateWriter;
use std::error::Error;
use std::ops::{AddAssign, Deref, DerefMut, MulAssign};
use tolerance::{Tolerance, Tolerances, compute_error};

use super::State;
use super::state_vector::{StateVector, StateVectorTolerances};

/// A dynamic-sized vector type for use in ODE solvers.
///
/// Unlike `StateArray`, this type supports arbitrary lengths and stores its data in a `Vec<f64>`.
#[derive(Clone, Debug, Default)]
pub struct StateVectors {
    /// Internal storage for the vector values.
    value: Vec<StateVector>,
    n: usize,
}

impl StateVectors {
    /// Constructs a new `StateVector` from a `Vec<f64>`.
    ///
    /// # Arguments
    ///
    /// * `value` - A `Vec<f64>` representing the initial state.
    pub fn new() -> Self {
        Self {
            value: Vec::new(),
            n: 0,
        }
    }

    pub fn add_state_vector(&mut self, v: StateVector) {
        self.value.push(v);
        self.n = self.value.len();
    }
}

impl AddAssign<&Self> for StateVectors {
    /// Performs element-wise addition of two `StateVector`s.
    ///
    /// # Panics
    ///
    /// Panics if the vectors have different lengths.
    fn add_assign(&mut self, rhs: &Self) {
        if self.n != rhs.n {
            panic!("state vectors do not have same length")
        }
        for i in 0..self.n {
            self.value[i] += &rhs.value[i];
        }
    }
}

impl MulAssign<f64> for StateVectors {
    /// Multiplies each element in the vector by a scalar value.
    fn mul_assign(&mut self, rhs: f64) {
        for i in 0..self.n {
            self.value[i] *= rhs;
        }
    }
}

impl Integrable for StateVectors {
    /// The derivative of the state is the same type.
    type Derivative = Self;

    /// Tolerances per vector component.
    type Tolerance = StateVectorsTolerances;
}

impl Deref for StateVectors {
    type Target = Vec<StateVector>;

    /// Provides access to the underlying `Vec<f64>` (read-only).
    fn deref(&self) -> &Self::Target {
        &self.value
    }
}

impl DerefMut for StateVectors {
    /// Provides mutable access to the underlying `Vec<f64>`.
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.value
    }
}

/// Stores optional component-wise tolerance rules for a `StateVector`.
///
/// If an entry is `None`, default relative and absolute tolerances are used for that component.
#[derive(Default)]
pub struct StateVectorsTolerances(Vec<StateVectorTolerances>);

impl Tolerance for StateVectorsTolerances {
    type State = StateVectors;

    /// Computes the root-mean-square error between estimated and actual states.
    ///
    /// Each element uses either a user-defined `Tolerances` object or falls back to the provided
    /// relative and absolute tolerances.
    ///
    /// # Arguments
    ///
    /// * `x` - Current state vector.
    /// * `x_prev` - Previous state vector.
    /// * `x_tilde` - Estimated state vector.
    /// * `rel_tol` - Default relative tolerance.
    /// * `abs_tol` - Default absolute tolerance.
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

        for (i, tol) in self.0.iter().enumerate() {
            let component_error =
                tol.compute_error(&x[i], &x_prev[i], &x_tilde[i], rel_tol, abs_tol);
            sum_squared_errors += component_error * component_error;
        }

        (sum_squared_errors / x.len() as f64).sqrt()
    }
}
