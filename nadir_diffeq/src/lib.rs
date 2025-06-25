use std::path::PathBuf;
use std::{error::Error, fmt::Debug};

pub mod events;
pub mod monte_carlo;
pub mod rk;
pub mod saving;
pub mod solvers;
pub mod state;
pub mod stepping;
pub mod tableau;

use crate::events::{PostSimEvent, PreSimEvent, SaveEvent};
use events::{ContinuousEvent, EventManager, PeriodicEvent};
use state::OdeState;
use uncertainty::Uncertainty;

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
    events: EventManager<Model, State>,
    model: Model,
    monte_carlo: Option<usize>,
    save_folder: Option<PathBuf>,
}

impl<Model, State> OdeProblem<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    /// Creates a new `OdeProblem` instance with the specified configuration.    
    pub fn new(model: Model) -> Self {
        Self {
            model,
            events: EventManager::new(),
            monte_carlo: None,
            save_folder: None,
        }
    }

    /// Adds a periodic event to the simulation.
    pub fn with_periodic_event(mut self, event: PeriodicEvent<Model, State>) -> Self {
        self.events
            .add_periodic(event);
        self
    }

    // Adds a presim event to the simulation.
    pub fn with_presim_event(mut self, event: PreSimEvent<Model, State>) -> Self {
        self.events
            .add_presim(event);
        self
    }

    /// Adds a postsim event to the simulation.
    pub fn with_postsim_event(mut self, event: PostSimEvent<Model>) -> Self {
        self.events
            .add_postsim(event);
        self
    }

    /// Adds a continuous event to the simulation.
    pub fn with_continuous_event(mut self, event: ContinuousEvent<Model, State>) -> Self {
        self.events
            .add_continuous(event);
        self
    }

    /// Adds the ability to save results to a folder on disk
    pub fn with_saving(mut self, save_folder: PathBuf) -> Self {
        self.save_folder = Some(save_folder);
        self
    }

    /// Adds a continuous event to the simulation.
    pub fn with_save_event(mut self, event: SaveEvent<Model, State>) -> Self {
        if self
            .save_folder
            .is_some()
        {
            self.events
                .add_save(event);
            self
        } else {
            panic!("need to call with_saving() before with_save_event() to enable saving")
        }
    }
}

/// In order to run a monte carlo simulation, the model must implement Uncertainty
impl<Model, State> OdeProblem<Model, State>
where
    Model: OdeModel<State = State> + Uncertainty,
    State: OdeState,
{
    pub fn with_monte_carlo(mut self, n: usize) -> Self {
        self.monte_carlo = Some(n);
        self
    }
}
