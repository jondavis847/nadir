use std::path::PathBuf;
pub mod events;
pub mod model;
pub mod monte_carlo;
pub mod rk;
pub mod saving;
pub mod solvers;
pub mod state;
pub mod stepping;
pub mod tableau;

use crate::events::{PostSimEvent, PreSimEvent, SaveEvent};
use crate::model::OdeModel;
use events::{ContinuousEvent, EventManager, PeriodicEvent};
use state::OdeState;

/// Container for a complete ODE simulation problem, including model, solver configuration,
/// step size control, result saving strategy, and event handling.
pub struct OdeProblem<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    events: EventManager<Model, State>,
    model: Model,
    save_folder: Option<PathBuf>,
}

impl<Model, State> OdeProblem<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    /// Creates a new `OdeProblem` instance with the specified configuration.    
    pub fn new(model: Model) -> Self {
        Self { model, events: EventManager::new(), save_folder: None }
    }

    /// Sets the events of the OdeProblem
    /// This will override any existing events!
    /// This is mostly just used for example in monte carlo to clone events from the builder to the instances
    /// You probably don't want to use this, just use the individualized event methods like with_periodic_events, etc.
    pub fn with_events(mut self, events: EventManager<Model, State>) -> Self {
        self.events = events;
        self
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
