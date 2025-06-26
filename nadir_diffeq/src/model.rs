use std::{error::Error, fmt::Debug};

use crate::state::OdeState;

/// Trait for defining a dynamical system model that can be numerically integrated.
///
/// Types implementing this trait must define how to compute the derivative (or RHS function)
/// of the ODE at a given time and state.
pub trait OdeModel: Debug {
    type State: OdeState;
    /// Compute the derivative at time `t` and state `state`, storing the result in `derivative`.
    fn f(
        &mut self,
        t: f64,
        state: &Self::State,
        derivative: &mut Self::State,
    ) -> Result<(), Box<dyn Error>>;
}

/// Allows users to have the state defined from the model rather than providing directly
pub trait StateFromModel {
    type State: OdeState;
    fn initial_state(&self) -> Self::State;
}

/// Allows users to have the state defined from the model rather than providing directly
/// Mutability is provided in case the user wants to store indices of state for submodel in the model, for example
pub trait StateFromModelMut {
    type State: OdeState;
    fn initial_state(&mut self) -> Self::State;
}
