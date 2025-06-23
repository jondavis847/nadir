use std::error::Error;

use crate::{
    OdeModel, OdeProblem,
    events::EventManager,
    rk::RungeKutta,
    saving::{MemoryResult, ResultStorage, SaveMethods, WriterManager},
    state::OdeState,
    stepping::{AdaptiveStepControl, StepMethods},
    tableau::ButcherTableau,
};

pub struct Solver {
    save_method: SaveMethods,
    solver_method: SolverMethods,
    step_method: StepMethods,
}

impl Default for Solver {
    fn default() -> Self {
        Self {
            save_method: SaveMethods::Memory,
            solver_method: SolverMethods::Explicit(ExplicitMethods::RungeKutta(
                RungeKuttaMethods::Tsit5,
            )),
            step_method: StepMethods::Adaptive(AdaptiveStepControl::default()),
        }
    }
}

impl Solver {
    pub fn new(
        solver_method: SolverMethods,
        step_method: StepMethods,
    ) -> Result<Self, Box<dyn Error>> {
        // handle inappropriate combos
        match &solver_method {
            SolverMethods::Explicit(method) => match method {
                ExplicitMethods::RungeKutta(method) => match method {
                    RungeKuttaMethods::Rk4 => match step_method {
                        StepMethods::Adaptive(_) => {
                            return Err("RK4 cannot be used with adaptive step methods".into());
                        }
                        _ => {}
                    },
                    _ => {}
                },
            },
            _ => {}
        }
        Ok(Self {
            solver_method,
            step_method,
            save_method: SaveMethods::Memory,
        })
    }

    pub fn solve<Model: OdeModel<State = State>, State: OdeState>(
        &mut self,
        mut problem: OdeProblem<Model, State>,
        x0: State,
        tspan: (f64, f64),
    ) -> Result<(), Box<dyn Error>> {
        let mut step_method = self.step_method.clone();

        // Initialize the manager for writing results to a file
        let mut writer_manager = if let Some(save_folder) = &problem.save_folder {
            let mut writer_manager = WriterManager::new();
            // Initialize the manager with the user provided builders
            for event in &mut problem.events.save_events {
                (event.init_fn)(&mut problem.model, &x0, &mut writer_manager);
            }
            // Initialize the writers from the builders
            writer_manager.initialize(save_folder)?;
            Some(writer_manager)
        } else {
            None
        };

        // Preallocate memory for result storage if needed
        let mut result = match self.save_method {
            SaveMethods::Memory => {
                match &self.step_method {
                    StepMethods::Adaptive(adaptive) => {
                        let n = if let Some(max_dt) = &adaptive.max_dt {
                            ((tspan.1 - tspan.0) / max_dt).ceil() as usize
                        } else {
                            // Default conservative allocation: 1 save per second
                            (tspan.1 - tspan.0).ceil() as usize
                        };
                        ResultStorage::Memory(MemoryResult::new(n))
                    }
                    StepMethods::Fixed(fixed) => {
                        let n = ((tspan.1 - tspan.0) / fixed.dt).ceil() as usize;
                        ResultStorage::Memory(MemoryResult::new(n))
                    }
                }
            }
            _ => ResultStorage::None,
        };

        // process any presim events
        for event in &problem.events.presim_events {
            (event.f)(&mut problem.model, &x0, tspan.0, &writer_manager)?;
        }

        self.solver_method.solve(
            &mut problem.model,
            &x0,
            tspan,
            &mut step_method,
            &mut problem.events,
            &mut result,
            &mut writer_manager,
        )
    }
}

pub enum SolverMethods {
    Explicit(ExplicitMethods),
    Implicit,
    MultiStep,
}

impl SolverMethods {
    fn solve<Model, State>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        step_method: &mut StepMethods,
        events: &mut EventManager<Model, State>,
        result: &mut ResultStorage<State>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState,
    {
        match self {
            SolverMethods::Explicit(method) => method.solve(
                model,
                x0,
                tspan,
                step_method,
                events,
                result,
                writer_manager,
            ),
            _ => todo!(),
        }
    }
}

pub enum ExplicitMethods {
    RungeKutta(RungeKuttaMethods),
}

impl ExplicitMethods {
    fn solve<Model, State>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        step_method: &mut StepMethods,
        events: &mut EventManager<Model, State>,
        result: &mut ResultStorage<State>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState,
    {
        match self {
            ExplicitMethods::RungeKutta(method) => method.solve(
                model,
                x0,
                tspan,
                step_method,
                events,
                result,
                writer_manager,
            ),
        }
    }
}

/// Enum representing the available solvers supported by the framework.
pub enum RungeKuttaMethods {
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

impl RungeKuttaMethods {
    pub fn solve<State, Model>(
        &self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut StepMethods,
        events: &mut EventManager<Model, State>,
        result: &mut ResultStorage<State>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState,
    {
        match self {
            RungeKuttaMethods::DoPri45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::DORMANDPRINCE45);
                solver.solve_adaptive(
                    model,
                    x0,
                    tspan,
                    controller,
                    events,
                    result,
                    writer_manager,
                )
            }
            RungeKuttaMethods::New45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::NEW45);
                solver.solve(
                    model,
                    x0,
                    tspan,
                    controller,
                    events,
                    result,
                    writer_manager,
                )
            }
            RungeKuttaMethods::Rk4 => {
                let mut solver = RungeKutta::new(ButcherTableau::<4, 4>::RK4);
                solver.solve(
                    model,
                    x0,
                    tspan,
                    controller,
                    events,
                    result,
                    writer_manager,
                )
            }
            RungeKuttaMethods::Tsit5 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::TSITOURAS5);
                solver.solve(
                    model,
                    x0,
                    tspan,
                    controller,
                    events,
                    result,
                    writer_manager,
                )
            }
            RungeKuttaMethods::Verner6 => {
                let mut solver = RungeKutta::new(ButcherTableau::<6, 9>::VERNER6);
                solver.solve(
                    model,
                    x0,
                    tspan,
                    controller,
                    events,
                    result,
                    writer_manager,
                )
            }
            RungeKuttaMethods::Verner9 => {
                let mut solver = RungeKutta::new(ButcherTableau::<9, 26>::VERNER9);
                solver.solve(
                    model,
                    x0,
                    tspan,
                    controller,
                    events,
                    result,
                    writer_manager,
                )
            }
        }
    }
}
