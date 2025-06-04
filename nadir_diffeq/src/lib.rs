use std::path::PathBuf;
use std::{error::Error, fmt::Debug};

/// Submodules for core ODE system components.
pub mod events;
pub mod rk;
pub mod saving;
pub mod solvers;
pub mod state;
pub mod stepping;
pub mod tableau;

use crate::events::{PostSimEvent, SaveEvent};
use crate::saving::WriterManager;
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
    save_folder: Option<PathBuf>,
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
            save_folder: None,
        }
    }

    /// Adds a periodic event to the simulation.
    pub fn with_periodic_event(mut self, event: PeriodicEvent<Model, State>) -> Self {
        self.events.add_periodic(event);
        self
    }

    /// Adds a periodic event to the simulation.
    pub fn with_postsim_event(mut self, event: PostSimEvent<Model>) -> Self {
        self.events.add_postsim(event);
        self
    }

    /// Adds a continuous event to the simulation.
    pub fn with_continuous_event(mut self, event: ContinuousEvent<Model, State>) -> Self {
        self.events.add_continuous(event);
        self
    }

    pub fn with_saving(mut self, save_folder: PathBuf) -> Self {
        self.save_folder = Some(save_folder);
        self
    }

    /// Adds a continuous event to the simulation.
    pub fn with_save_event(mut self, event: SaveEvent<Model, State>) -> Self {
        if self.save_folder.is_some() {
            self.events.add_save(event);
            self
        } else {
            panic!("need to call with_saving() before with_save_event() to enable saving")
        }
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
        // Initialize the manager for writing results to a file
        let mut writer_manager = if let Some(save_folder) = &self.save_folder {
            let mut writer_manager = WriterManager::new();
            // Initialize the manager with the user provided builders
            for event in &mut self.events.save_events {
                (event.init_fn)(&mut self.model, x0, &mut writer_manager);
            }
            // Initialize the writers from the builders
            writer_manager.initialize(save_folder)?;
            Some(writer_manager)
        } else {
            None
        };

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
            &mut writer_manager,
        )?;

        // process any postsim events
        for event in &self.events.postsim_events {
            (event.postsim_fn)(&self.model, &writer_manager);
        }

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
        // Initialize the manager for writing results to a file
        let mut writer_manager = if let Some(save_folder) = &self.save_folder {
            let mut writer_manager = WriterManager::new();
            // Initialize the manager with the user provided builders
            for event in &mut self.events.save_events {
                (event.init_fn)(&mut self.model, x0, &mut writer_manager);
            }
            // Initialize the writers from the builders
            writer_manager.initialize(save_folder)?;
            Some(writer_manager)
        } else {
            None
        };

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
            &mut writer_manager,
        )?;

        // Finalize and return the results
        result.truncate()?;
        Ok(result)
    }
}
