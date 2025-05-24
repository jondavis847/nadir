use std::{error::Error, fmt::Debug};

/// Submodules for core ODE system components.
pub mod events;
pub mod rk;
pub mod saving;
pub mod state;
pub mod stepping;
pub mod tableau;

use crate::rk::RungeKutta;
use crate::tableau::ButcherTableau;
use events::{ContinuousEvent, EventManager, PeriodicEvent};
use saving::{MemoryResult, ResultStorage, SaveMethod};
use state::Integrable;
use stepping::StepMethod;

/// Trait for defining a dynamical system model that can be numerically integrated.
///
/// Types implementing this trait must define how to compute the derivative (or RHS function)
/// of the ODE at a given time and state.
pub trait OdeModel<State>: Debug
where
    State: Integrable,
{
    /// Compute the derivative at time `t` and state `state`, storing the result in `derivative`.
    fn f(
        &mut self,
        t: f64,
        state: &State,
        derivative: &mut State::Derivative,
    ) -> Result<(), Box<dyn Error>>;
}

/// Enum representing the available solvers supported by the framework.
pub enum Solver {
    /// Dormand-Prince 4(5) method.
    DoPri45,
    /// Tsitouras new 4(5) method variant.
    New45,
    /// Classical Runge-Kutta 4th-order method.
    Rk4,
    /// Tsitouras 5(4) method.
    Tsit5,
    /// Verner’s 6th-order embedded method.
    Verner6,
    /// Verner’s 9th-order embedded method.
    Verner9,
}

/// Container for a complete ODE simulation problem, including model, solver configuration,
/// step size control, result saving strategy, and event handling.
pub struct OdeProblem<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable + 'static,
{
    model: Model,
    solver: Solver,
    step_method: StepMethod,
    save_method: SaveMethod<State>,
    events: EventManager<Model, State>,
}

impl<Model, State> OdeProblem<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable,
{
    /// Creates a new `OdeProblem` instance with the specified configuration.
    ///
    /// # Panics
    ///
    /// Panics if an adaptive step method is used with `Rk4`, which does not support adaptivity.
    pub fn new(
        model: Model,
        solver: Solver,
        step_method: StepMethod,
        save_method: SaveMethod<State>,
    ) -> Self {
        match (&solver, &step_method) {
            (Solver::Rk4, StepMethod::Adaptive(_)) => {
                panic!("Rk4 solver does not support adaptive step size")
            }
            _ => {}
        }

        Self {
            model,
            solver,
            save_method,
            step_method,
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
    pub fn solve(
        &mut self,
        x0: &State,
        tspan: (f64, f64),
    ) -> Result<ResultStorage<State>, Box<dyn Error>> {
        // Preallocate memory for result storage if needed
        let mut result = match &self.save_method {
            SaveMethod::Memory => {
                let n = match self.step_method {
                    StepMethod::Fixed(controller) => {
                        ((tspan.1 - tspan.0) / controller.dt).ceil() as usize
                    }
                    StepMethod::Adaptive(controller) => {
                        if let Some(max_dt) = &controller.max_dt {
                            ((tspan.1 - tspan.0) / max_dt).ceil() as usize
                        } else {
                            // Default conservative allocation: 1 save per second
                            (tspan.1 - tspan.0).ceil() as usize
                        }
                    }
                };
                ResultStorage::Memory(MemoryResult::<State>::new(n))
            }
            SaveMethod::File(builder) => ResultStorage::File(builder.to_writer()?),
            _ => ResultStorage::None,
        };

        // Dispatch to the appropriate solver
        match self.solver {
            Solver::DoPri45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::DORMANDPRINCE45);
                solver.solve(
                    &mut self.model,
                    x0,
                    tspan,
                    &mut self.step_method,
                    &mut self.events,
                    &mut result,
                );
            }
            Solver::New45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::NEW45);
                solver.solve(
                    &mut self.model,
                    x0,
                    tspan,
                    &mut self.step_method,
                    &mut self.events,
                    &mut result,
                );
            }
            Solver::Rk4 => {
                let mut solver = RungeKutta::new(ButcherTableau::<4, 4>::RK4);
                solver.solve(
                    &mut self.model,
                    x0,
                    tspan,
                    &mut self.step_method,
                    &mut self.events,
                    &mut result,
                );
            }
            Solver::Tsit5 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::TSITOURAS5);
                solver.solve(
                    &mut self.model,
                    x0,
                    tspan,
                    &mut self.step_method,
                    &mut self.events,
                    &mut result,
                );
            }
            Solver::Verner6 => {
                let mut solver = RungeKutta::new(ButcherTableau::<6, 9>::VERNER6);
                solver.solve(
                    &mut self.model,
                    x0,
                    tspan,
                    &mut self.step_method,
                    &mut self.events,
                    &mut result,
                );
            }
            Solver::Verner9 => {
                let mut solver = RungeKutta::new(ButcherTableau::<9, 26>::VERNER9);
                solver.solve(
                    &mut self.model,
                    x0,
                    tspan,
                    &mut self.step_method,
                    &mut self.events,
                    &mut result,
                );
            }
        }

        // Finalize and return the results
        result.truncate()?;
        Ok(result)
    }
}
