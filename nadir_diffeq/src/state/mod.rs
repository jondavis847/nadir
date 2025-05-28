//! A generic CSV state writer for types implementing the `Integrable` trait.
//! This module allows defining how a state is formatted into a CSV-compatible
//! row and then writing it out incrementally.

use std::{
    error::Error,
    fmt::Debug,
    ops::{AddAssign, MulAssign},
};
use tolerance::Tolerance;

use crate::saving::StateWriter;

pub mod state_array;
pub mod state_vector;

pub trait State: Integrable + Default + Clone + Sized + 'static {
    fn headers(_ncols: usize) -> Vec<String> {
        Vec::new()
    }
    fn write_record(&self, _t: f64, _writer: &mut StateWriter) -> Result<(), Box<dyn Error>> {
        panic!(
            "Writing not implemented for this state. Implement 'write_headers' and 'write_record' methods to enable writing."
        );
    }
}

/// Trait representing an integrable state for use in ODE solvers.
///
/// Types implementing this trait must support arithmetic operations,
/// cloning, and formatting for debugging. The associated `Derivative`
/// type represents the derivative of the state, and `Tolerance` is used
/// for controlling adaptive solver behavior.
pub trait Integrable: MulAssign<f64>
where
    for<'a> Self: AddAssign<&'a Self> + AddAssign<&'a Self::Derivative>,
{
    /// The derivative of the state, used in ODE computation.
    type Derivative: Clone + MulAssign<f64> + Sized + Default + Debug;

    /// The tolerance model associated with the state, used for error estimation.
    type Tolerance: Tolerance<State = Self>;
}
