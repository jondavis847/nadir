use std::fmt::Write;
use std::ops::{AddAssign, Deref, DerefMut, MulAssign};
use tolerance::{Tolerance, Tolerances, compute_error};

use super::StateWriterBuilder;
use crate::Integrable;

/// A dynamic-sized vector type for use in ODE solvers.
///
/// Unlike `StateArray`, this type supports arbitrary lengths and stores its data in a `Vec<f64>`.
#[derive(Clone, Debug, Default)]
pub struct StateVector {
    /// Internal storage for the vector values.
    value: Vec<f64>,
    /// Cached length of the vector to avoid repeated calls to `.len()`.
    n: usize,
}

impl StateVector {
    /// Constructs a new `StateVector` from a `Vec<f64>`.
    ///
    /// # Arguments
    ///
    /// * `value` - A `Vec<f64>` representing the initial state.
    pub fn new(value: Vec<f64>) -> Self {
        let n = value.len();
        Self { value, n }
    }
}

impl AddAssign<&Self> for StateVector {
    /// Performs element-wise addition of two `StateVector`s.
    ///
    /// # Panics
    ///
    /// Panics if the vectors have different lengths.
    fn add_assign(&mut self, rhs: &Self) {
        if self.n != rhs.n {
            panic!("state vectors do not have same length")
        }
        for i in 0..self.len() {
            self.value[i] += rhs.value[i];
        }
    }
}

impl MulAssign<f64> for StateVector {
    /// Multiplies each element in the vector by a scalar value.
    fn mul_assign(&mut self, rhs: f64) {
        for i in 0..self.n {
            self.value[i] *= rhs;
        }
    }
}

impl Integrable for StateVector {
    /// The derivative of the state is the same type.
    type Derivative = Self;

    /// Tolerances per vector component.
    type Tolerance = StateVectorTolerances;

    /// Returns a `StateWriterBuilder` for logging state data to a CSV file.
    ///
    /// # Arguments
    ///
    /// * `path` - Path to the output file.
    fn writer(path: std::path::PathBuf) -> StateWriterBuilder<Self> {
        StateWriterBuilder::new(path, |t, x: &Self, buffer: &mut Vec<String>| {
            if buffer.len() != x.n {
                buffer.resize(x.n + 1, String::new());
            }
            write!(buffer[0], "{}", t)?;
            for i in 0..x.len() {
                write!(buffer[i + 1], "{}", x[i])?;
            }
            Ok(())
        })
    }
}

impl Deref for StateVector {
    type Target = Vec<f64>;

    /// Provides access to the underlying `Vec<f64>` (read-only).
    fn deref(&self) -> &Self::Target {
        &self.value
    }
}

impl DerefMut for StateVector {
    /// Provides mutable access to the underlying `Vec<f64>`.
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.value
    }
}

/// Stores optional component-wise tolerance rules for a `StateVector`.
///
/// If an entry is `None`, default relative and absolute tolerances are used for that component.
#[derive(Default)]
pub struct StateVectorTolerances(Vec<Option<Tolerances>>);

impl Tolerance for StateVectorTolerances {
    type State = StateVector;

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

        for (i, tol) in self.0.iter().enumerate() {
            let component_error = if let Some(tol) = tol {
                tol.compute_error(x.value[i], x_prev.value[i], x_tilde.value[i])
            } else {
                compute_error(
                    x.value[i],
                    x_prev.value[i],
                    x_tilde.value[i],
                    rel_tol,
                    abs_tol,
                )
            };

            sum_squared_errors += component_error * component_error;
        }

        (sum_squared_errors / x.len() as f64).sqrt()
    }
}
