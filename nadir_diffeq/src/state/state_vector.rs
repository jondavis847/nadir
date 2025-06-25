use crate::state::Adaptive;
use std::ops::{AddAssign, Deref, DerefMut, MulAssign};
use tolerance::compute_error;
use uncertainty::{UncertainValue, Uncertainty};

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
        self.value
            .extend_from_slice(&other.value);
        self.n = self
            .value
            .len();
    }

    /// Constructs a new `StateVector` with preallocated capcity.
    ///
    /// # Arguments
    ///
    /// * `n` - usize for the number of elements.
    pub fn with_capacity(n: usize) -> Self {
        Self { value: Vec::with_capacity(n), n }
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
            panic!(
                "length of rhs ({}) does not match length of self ({})",
                rhs.n, self.n
            )
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

impl Adaptive for StateVector {
    fn compute_error(&self, x_prev: &Self, x_tilde: &Self, abs_tol: f64, rel_tol: f64) -> f64 {
        let mut accum_error = 0.0;
        for i in 0..self.len() {
            let error = compute_error(
                self[i], x_prev[i], x_tilde[i], rel_tol, abs_tol,
            );
            accum_error += error * error;
        }
        (accum_error / self.len() as f64).sqrt()
    }
}

// /// Stores optional component-wise tolerance rules for a `StateVector`.
// ///
// /// If an entry is `None`, default relative and absolute tolerances are used for that component.
// #[derive(Default)]
// pub struct StateVectorTolerances(pub Vec<Tolerances>);

// impl StateVectorTolerances {
//     /// Computes the root-mean-square error between estimated and actual states.
//     ///
//     /// Each element uses either a user-defined `Tolerances` object or falls back to the provided
//     /// relative and absolute tolerances.
//     ///
//     /// # Arguments
//     ///
//     /// * `x` - Current state vector.
//     /// * `x_prev` - Previous state vector.
//     /// * `x_tilde` - Estimated state vector.
//     /// * `rel_tol` - Default relative tolerance.
//     /// * `abs_tol` - Default absolute tolerance.
//     fn compute_error(&self, x: &StateVector, x_prev: &StateVector, x_tilde: &StateVector) -> f64 {
//         if x.len() == 0 {
//             return 0.0;
//         }
//         let mut sum_squared_errors = 0.0;

//         for (i, tol) in self.0.iter().enumerate() {
//             let component_error = tol.compute_error(x.value[i], x_prev.value[i], x_tilde.value[i]);
//             sum_squared_errors += component_error * component_error;
//         }

//         (sum_squared_errors / x.len() as f64).sqrt()
//     }

//     pub fn from_config(config: &StateConfig, global_abs_tol: f64, global_rel_tol: f64) -> Self {
//         let global_tol = Tolerances::new(global_rel_tol, global_abs_tol);
//         let tolerances = config
//             .tolerances
//             .iter()
//             .map(|tol| {
//                 if let Some(tol) = tol {
//                     *tol
//                 } else {
//                     global_tol
//                 }
//             })
//             .collect();
//         Self(tolerances)
//     }
// }

#[derive(Clone)]
pub struct UncertainStateVector(pub Vec<UncertainValue>);

impl UncertainStateVector {
    pub fn len(&self) -> usize {
        self.0
            .len()
    }
}

impl Uncertainty for UncertainStateVector {
    type Output = StateVector;
    type Error = ();
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        let mut output = StateVector::new(vec![0.0; self.len()]);
        for (uncertain_val, out_val) in self
            .0
            .iter()
            .zip(output.iter_mut())
        {
            *out_val = uncertain_val.sample(nominal, rng);
        }
        Ok(output)
    }
}
