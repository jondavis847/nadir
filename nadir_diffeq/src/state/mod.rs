//! A generic CSV state writer for types implementing the `Integrable` trait.
//! This module allows defining how a state is formatted into a CSV-compatible
//! row and then writing it out incrementally.

use std::{
    fmt::Debug,
    ops::{AddAssign, MulAssign},
};
use tolerance::Tolerance;

use crate::saving::WritableState;

pub mod state_array;
pub mod state_vector;

pub trait State: Integrable + WritableState + Default + Clone + Sized + 'static {}

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
