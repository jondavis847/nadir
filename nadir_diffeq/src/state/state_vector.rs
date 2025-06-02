use std::ops::{AddAssign, Deref, DerefMut, MulAssign};
use tolerance::Tolerances;

use super::{OdeState, StateConfig};

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

    /// Extends a `StateVector` with the content of another.
    ///
    /// # Arguments
    ///
    /// * `n` - usize for the number of elements.
    pub fn extend(&mut self, other: &Self) {
        self.value.extend_from_slice(&other.value);
        self.n = self.value.len();
    }

    /// Constructs a new `StateVector` with preallocated capcity.
    ///
    /// # Arguments
    ///
    /// * `n` - usize for the number of elements.
    pub fn with_capacity(n: usize) -> Self {
        Self {
            value: Vec::with_capacity(n),
            n,
        }
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

impl OdeState for StateVector {
    /// The derivative type is the same as the state type.
    type Derivative = Self;

    /// Returns a default configuration for the `StateVector`.
    fn config() -> Result<StateConfig, Box<dyn std::error::Error>> {
        Ok(StateConfig {
            n: 0,
            tolerances: Vec::new(),
            writers: Vec::new(),
        })
    }

    /// Reads values from a `StateVector` into this state.
    fn read_vector(&mut self, x: &StateVector) {
        if self.n != x.len() {
            panic!("StateVector length does not match StateVector size");
        }
        self.value.clone_from(&x.value);
    }

    /// Writes values from this state into a `StateVector`.
    fn write_vector(&self, x: &mut StateVector) {
        if self.n != x.len() {
            x.resize(self.n, 0.0);
        }
        x.value.clone_from(&self.value);
    }
}

/// Stores optional component-wise tolerance rules for a `StateVector`.
///
/// If an entry is `None`, default relative and absolute tolerances are used for that component.
#[derive(Default)]
pub struct StateVectorTolerances(pub Vec<Tolerances>);

impl StateVectorTolerances {
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
    fn compute_error(&self, x: &StateVector, x_prev: &StateVector, x_tilde: &StateVector) -> f64 {
        if x.len() == 0 {
            return 0.0;
        }
        let mut sum_squared_errors = 0.0;

        for (i, tol) in self.0.iter().enumerate() {
            let component_error = tol.compute_error(x.value[i], x_prev.value[i], x_tilde.value[i]);
            sum_squared_errors += component_error * component_error;
        }

        (sum_squared_errors / x.len() as f64).sqrt()
    }

    pub fn from_config(config: &StateConfig, global_abs_tol: f64, global_rel_tol: f64) -> Self {
        let global_tol = Tolerances::new(global_rel_tol, global_abs_tol);
        let tolerances = config
            .tolerances
            .iter()
            .map(|tol| {
                if let Some(tol) = tol {
                    *tol
                } else {
                    global_tol
                }
            })
            .collect();
        Self(tolerances)
    }
}
