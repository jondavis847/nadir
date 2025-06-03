use std::{error::Error, fmt::Debug};

/// Submodules for core ODE system components.
pub mod events;
pub mod rk;
pub mod saving;
pub mod solvers;
pub mod state;
pub mod stepping;
pub mod tableau;

use crate::solvers::Solver;
use crate::state::Adaptive;
use crate::stepping::AdaptiveStepControl;
use crate::stepping::FixedStepControl;
use events::{ContinuousEvent, EventManager, PeriodicEvent};
use saving::{MemoryResult, ResultStorage, SaveMethod};
use state::OdeState;

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

/// Container for a complete ODE simulation problem, including model, solver configuration,
/// step size control, result saving strategy, and event handling.
pub struct OdeProblem<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    model: Model,
    events: EventManager<Model, State>,
}

impl<Model, State> OdeProblem<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    /// Creates a new `OdeProblem` instance with the specified configuration.
    ///
    /// # Panics
    ///
    /// Panics if an adaptive step method is used with `Rk4`, which does not support adaptivity.
    pub fn new(model: Model) -> Self {
        Self {
            model,
            events: EventManager::new(),
        }
    }

    /// Adds a periodic event to the simulation.
    pub fn with_event_periodic(mut self, event: PeriodicEvent<Model, State>) -> Self {
        self.events.add_periodic(event);
        self
    }

    /// Adds a continuous event to the simulation.
    pub fn with_event_continuous(mut self, event: ContinuousEvent<Model, State>) -> Self {
        self.events.add_continuous(event);
        self
    }

    pub fn solve_adaptive(
        &mut self,
        x0: &State,
        tspan: (f64, f64),
        mut step_control: AdaptiveStepControl,
        solver: Solver,
        save_method: SaveMethod,
    ) -> Result<ResultStorage<State>, Box<dyn Error>>
    where
        State: Adaptive,
    {
        // Preallocate memory for result storage if needed
        let mut result = match save_method {
            SaveMethod::Memory => {
                let n = if let Some(max_dt) = &step_control.max_dt {
                    ((tspan.1 - tspan.0) / max_dt).ceil() as usize
                } else {
                    // Default conservative allocation: 1 save per second
                    (tspan.1 - tspan.0).ceil() as usize
                };
                ResultStorage::Memory(MemoryResult::new(n))
            }
            _ => ResultStorage::None,
        };

        // Dispatch to the appropriate solver
        solver.solve_adaptive(
            &mut self.model,
            x0,
            tspan,
            &mut step_control,
            &mut self.events,
            &mut result,
        )?;

        // Finalize and return the results
        result.truncate()?;
        Ok(result)
    }

    /// Solves the ODE problem over the specified time span.
    ///
    /// This method internally selects the appropriate solver implementation based on the
    /// `Solver` enum, configures result storage, handles events, and integrates the system.
    ///
    /// # Arguments
    ///
    /// * `x0` - Initial state.
    /// * `tspan` - Tuple of start and end times.
    ///
    /// # Returns
    ///
    /// A result containing the populated `ResultStorage` (either in memory or to file).

    pub fn solve_fixed(
        &mut self,
        x0: &State,
        tspan: (f64, f64),
        dt: f64,
        solver: Solver,
        save_method: SaveMethod,
    ) -> Result<ResultStorage<State>, Box<dyn Error>> {
        // Preallocate memory for result storage if needed
        let mut result = match save_method {
            SaveMethod::Memory => {
                let n = ((tspan.1 - tspan.0) / dt).ceil() as usize;
                ResultStorage::Memory(MemoryResult::new(n))
            }
            _ => ResultStorage::None,
        };

        let mut controller = FixedStepControl::new(dt);

        solver.solve_fixed(
            &mut self.model,
            x0,
            tspan,
            &mut controller,
            &mut self.events,
            &mut result,
        )?;

        // Finalize and return the results
        result.truncate()?;
        Ok(result)
    }
}
