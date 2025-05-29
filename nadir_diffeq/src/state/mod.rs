//! A generic CSV state writer for types implementing the `Integrable` trait.
//! This module allows defining how a state is formatted into a CSV-compatible
//! row and then writing it out incrementally.

use state_vector::StateVector;
use std::{
    error::Error,
    fmt::Debug,
    ops::{AddAssign, MulAssign},
};
use tolerance::{Tolerance, Tolerances};

use crate::saving::{StateWriter, StateWriterBuilder};

//pub mod state_array;
pub mod state_vector;
//pub mod state_vectors;

pub struct StateConfig {
    pub n: usize,
    pub tolerances: Vec<Option<Tolerances>>,
    writers: Vec<StateWriterBuilder>,
}

impl StateConfig {
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

    /// sets all of the tolerances of the statevector representation of the state
    /// None values will default to globally set abs_tol and rel_tol in the solver
    pub fn with_headers(mut self, tol: Vec<Option<Tolerances>>) -> Result<Self, Box<dyn Error>> {
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

pub trait State: Clone + Default + Clone + Sized + 'static {
    type Derivative: Clone + Default + Clone + Sized + 'static;

    fn config() -> StateConfig;
    /// Read in the values from a StateVector to your custom State
    fn read_state(&mut self, x: StateVector);
    /// Write values from your custom State derivative into the integration buffer
    fn write_derivative(&self, dx: &mut StateVector);
}
