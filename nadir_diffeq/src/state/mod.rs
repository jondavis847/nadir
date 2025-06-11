//! A generic CSV state writer for types implementing the `Integrable` trait.
//! This module allows defining how a state is formatted into a CSV-compatible
//! row and then writing it out incrementally.

use std::error::Error;
use std::fmt::Debug;
use std::ops::{AddAssign, MulAssign};

use tolerance::Tolerances;

use crate::saving::StateWriterBuilder;

pub mod state_array;
pub mod state_vector;
//pub mod state_vectors;

pub struct StateConfig {
    pub n: usize,
    pub tolerances: Vec<Option<Tolerances>>,
    pub writers: Vec<StateWriterBuilder>,
}

impl StateConfig {
    pub fn new(n: usize) -> Self {
        if n == 0 {
            panic!("n must be greater than 0 for StateConfig");
        }
        Self {
            n,
            tolerances: vec![None; n],
            writers: Vec::new(),
        }
    }

    /// sets the tolerances of a single element in the statevector representation of the state
    /// None values will default to globally set abs_tol and rel_tol in the solver
    pub fn with_tolerance(
        mut self,
        element: usize,
        tol: Option<Tolerances>,
    ) -> Result<Self, Box<dyn Error>> {
        if element >= self.n {
            return Err(format!(
                "element ({}) cannot be greater than n ({}) for StateConfig",
                element, self.n
            )
            .into());
        }
        self.tolerances[element] = tol;
        Ok(self)
    }

    /// sets all of the tolerances of the statevector representation of the state
    /// None values will default to globally set abs_tol and rel_tol in the solver
    pub fn with_tolerances(mut self, tol: Vec<Option<Tolerances>>) -> Result<Self, Box<dyn Error>> {
        if tol.len() != self.n {
            return Err(format!(
                "length of tol ({}) must be equal to n ({}) for StateConfig",
                tol.len(),
                self.n
            )
            .into());
        }
        self.tolerances = tol;
        Ok(self)
    }
}

pub trait OdeState: Debug + Clone + Default + Clone + Sized + MulAssign<f64> + 'static
where
    for<'a> Self: AddAssign<&'a Self>,
{
}

impl<T> OdeState for T
where
    T: Debug + Clone + Default + Sized + MulAssign<f64> + 'static,
    for<'a> T: AddAssign<&'a T>,
{
}
pub trait Adaptive {
    fn compute_error(&self, _x_prev: &Self, _x_tilde: &Self, _abs_tol: f64, _rel_tol: f64) -> f64;
}
