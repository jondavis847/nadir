use std::{
    fmt::Write,
    ops::{AddAssign, Deref, DerefMut, MulAssign},
    path::PathBuf,
};

use tolerance::{Tolerance, Tolerances, compute_error};

use super::StateWriterBuilder;
use crate::Integrable;

/// A fixed-size array wrapper representing a generic state vector with `N` f64 components.
///
/// This type is commonly used as a concrete state for ODE solvers.
#[derive(Clone, Copy, Debug)]
pub struct StateArray<const N: usize>([f64; N]);

impl<const N: usize> StateArray<N> {
    /// Constructs a new `StateArray` from an array of `f64`.
    ///
    /// # Arguments
    ///
    /// * `array` - An array of `f64` values representing the state.
    pub fn new(array: [f64; N]) -> Self {
        Self(array)
    }
}

impl<const N: usize> Default for StateArray<N> {
    /// Creates a `StateArray` with all elements initialized to zero.
    fn default() -> Self {
        Self([0.0; N])
    }
}

impl<const N: usize> AddAssign<&Self> for StateArray<N> {
    /// Adds each element from the right-hand side into `self` in-place.
    ///
    /// This operation is element-wise.
    fn add_assign(&mut self, rhs: &Self) {
        for i in 0..N {
            self.0[i] += rhs.0[i];
        }
    }
}

impl<const N: usize> MulAssign<f64> for StateArray<N> {
    /// Multiplies each element of the array in-place by the given scalar.
    fn mul_assign(&mut self, rhs: f64) {
        for i in 0..N {
            self.0[i] *= rhs;
        }
    }
}

impl<const N: usize> Integrable for StateArray<N> {
    /// The derivative is represented by the same type as the state.
    type Derivative = Self;

    /// Per-component tolerances for error control.
    type Tolerance = StateArrayTolerances<N>;

    /// Returns a `StateWriterBuilder` that outputs time and state values to CSV.
    ///
    /// # Arguments
    ///
    /// * `path` - Path to the output CSV file.
    fn writer(path: PathBuf) -> Vec<StateWriterBuilder<Self>> {
        vec![StateWriterBuilder::<Self>::new(
            path,
            |t: f64, x: &Self, buffer: &mut Vec<String>| {
                buffer[0].clear();
                write!(buffer[0], "{}", t).unwrap();
                for i in 0..N {
                    write!(buffer[i + 1], "{}", x[i]).unwrap();
                }
                Ok(())
            },
        )]
    }
}

impl<const N: usize> Deref for StateArray<N> {
    type Target = [f64; N];

    /// Dereferences the `StateArray` to access the underlying array.
    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize> DerefMut for StateArray<N> {
    /// Mutable dereference to the underlying array.
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

/// Stores optional per-element tolerance strategies for a `StateArray`.
///
/// If an entry is `None`, default absolute and relative tolerances are used for that component.
pub struct StateArrayTolerances<const N: usize>([Option<Tolerances>; N]);

impl<const N: usize> Tolerance for StateArrayTolerances<N> {
    type State = StateArray<N>;

    /// Computes the root-mean-square error between the estimated and true state.
    ///
    /// Each element uses its own tolerance configuration if provided. Falls back to `rel_tol` and
    /// `abs_tol` if no per-component tolerance is available.
    ///
    /// # Arguments
    ///
    /// * `x` - The current state.
    /// * `x_prev` - The previous state.
    /// * `x_tilde` - The error-estimate state.
    /// * `rel_tol` - Default relative tolerance.
    /// * `abs_tol` - Default absolute tolerance.
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

        for (i, tol) in self.0.iter().enumerate() {
            let component_error = if let Some(tol) = tol {
                tol.compute_error(x.0[i], x_prev.0[i], x_tilde.0[i])
            } else {
                compute_error(x.0[i], x_prev.0[i], x_tilde.0[i], rel_tol, abs_tol)
            };

            sum_squared_errors += component_error * component_error;
        }

        (sum_squared_errors / N as f64).sqrt()
    }
}

impl<const N: usize> Default for StateArrayTolerances<N> {
    /// Creates a new `StateArrayTolerances` with no component-specific tolerances.
    fn default() -> Self {
        Self([None; N])
    }
}
