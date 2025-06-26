use std::error::Error;

use crate::{
    OdeModel, OdeProblem,
    events::EventManager,
    model::{StateFromModel, StateFromModelMut},
    rk::RungeKutta,
    saving::{MemoryResult, SaveMethods, WriterManager},
    state::{Adaptive, OdeState},
    stepping::{AdaptiveStepControl, FixedStepControl},
    tableau::ButcherTableau,
};

#[derive(Clone, Copy)]
pub struct OdeSolver {
    save_method: SaveMethods,
    solver_method: SolverMethods,
}

impl Default for OdeSolver {
    fn default() -> Self {
        Self {
            save_method: SaveMethods::Memory,
            solver_method: SolverMethods::Explicit(ExplicitMethods::RungeKutta(
                RungeKuttaMethods::Tsit5,
            )),
        }
    }
}

impl OdeSolver {
    pub fn new(solver_method: SolverMethods) -> Self {
        Self { solver_method, save_method: SaveMethods::Memory }
    }

    pub fn solve_adaptive<Model: OdeModel<State = State>, State: OdeState + Adaptive>(
        &self,
        mut problem: OdeProblem<Model, State>,
        x0: State,
        tspan: (f64, f64),
        mut controller: AdaptiveStepControl,
    ) -> Result<Option<MemoryResult<State>>, Box<dyn Error>> {
        // handle inappropriate combos
        match &self.solver_method {
            SolverMethods::Explicit(method) => match method {
                ExplicitMethods::RungeKutta(method) => match method {
                    RungeKuttaMethods::Rk4 => {
                        return Err("RK4 cannot be used with adaptive step methods".into());
                    }
                    _ => {}
                },
            },
            _ => {}
        }

        // Initialize the manager for writing results to a file
        let mut writer_manager = self.initialize_writer(&mut problem, &x0)?;
        // Preallocate memory for result storage if needed
        let mut result = self.initialize_adaptive_result(&x0, &tspan, &controller);

        // process any presim events
        for event in &problem
            .events
            .presim_events
        {
            (event.f)(
                &mut problem.model,
                &x0,
                tspan.0,
                &writer_manager,
            )?;
        }

        self.solver_method
            .solve_adaptive(
                &mut problem.model,
                &x0,
                tspan,
                &mut controller,
                &mut problem.events,
                &mut result,
                &mut writer_manager,
            )?;

        // process any postsim events
        for event in &problem
            .events
            .postsim_events
        {
            (event.f)(
                &mut problem.model,
                &writer_manager,
            );
        }

        // Finalize and return the results
        if let Some(result) = &mut result {
            result.truncate();
        }

        Ok(result)
    }

    pub fn solve_fixed<Model: OdeModel<State = State>, State: OdeState>(
        &self,
        mut problem: OdeProblem<Model, State>,
        x0: State,
        tspan: (f64, f64),
        dt: f64,
    ) -> Result<Option<MemoryResult<State>>, Box<dyn Error>> {
        let mut controller = FixedStepControl::new(dt);
        // Initialize the manager for writing results to a file
        let mut writer_manager = self.initialize_writer(&mut problem, &x0)?;
        // Preallocate memory for result storage if needed
        let mut result = self.initialize_fixed_result(&x0, &tspan, &controller);

        // process any presim events
        for event in &problem
            .events
            .presim_events
        {
            (event.f)(
                &mut problem.model,
                &x0,
                tspan.0,
                &writer_manager,
            )?;
        }

        self.solver_method
            .solve_fixed(
                &mut problem.model,
                &x0,
                tspan,
                &mut controller,
                &mut problem.events,
                &mut result,
                &mut writer_manager,
            )?;

        // process any postsim events
        for event in &problem
            .events
            .postsim_events
        {
            (event.f)(
                &mut problem.model,
                &writer_manager,
            );
        }

        // Finalize and return the results
        if let Some(result) = &mut result {
            result.truncate();
        }

        Ok(result)
    }

    /// Solves the ODE problem, but with the initial state coming from the model rather than provided directly
    pub fn solve_model_adaptive<Model, State>(
        &self,
        problem: OdeProblem<Model, State>,
        tspan: (f64, f64),
        controller: AdaptiveStepControl,
    ) -> Result<Option<MemoryResult<State>>, Box<dyn Error>>
    where
        Model: OdeModel<State = State> + StateFromModel<State = State>,
        State: OdeState + Adaptive,
    {
        let x0 = problem
            .model
            .initial_state();
        self.solve_adaptive(problem, x0, tspan, controller)
    }

    /// Solves the ODE problem, but with the initial state coming from the model rather than provided directly
    /// Mutability allows the user to store indices of submodel state vector elements in the model, for example
    pub fn solve_model_adaptive_mut<Model, State>(
        &self,
        mut problem: OdeProblem<Model, State>,
        tspan: (f64, f64),
        controller: AdaptiveStepControl,
    ) -> Result<Option<MemoryResult<State>>, Box<dyn Error>>
    where
        Model: OdeModel<State = State> + StateFromModelMut<State = State>,
        State: OdeState + Adaptive,
    {
        let x0 = problem
            .model
            .initial_state();
        self.solve_adaptive(problem, x0, tspan, controller)
    }

    /// Solves the ODE problem, but with the initial state coming from the model rather than provided directly
    pub fn solve_model_fixed<Model, State>(
        &self,
        problem: OdeProblem<Model, State>,
        tspan: (f64, f64),
        dt: f64,
    ) -> Result<Option<MemoryResult<State>>, Box<dyn Error>>
    where
        Model: OdeModel<State = State> + StateFromModel<State = State>,
        State: OdeState,
    {
        let x0 = problem
            .model
            .initial_state();
        self.solve_fixed(problem, x0, tspan, dt)
    }

    /// Solves the ODE problem, but with the initial state coming from the model rather than provided directly
    /// Mutability allows the user to store indices of submodel state vector elements in the model, for example
    pub fn solve_model_fixed_mut<Model, State>(
        &self,
        mut problem: OdeProblem<Model, State>,
        tspan: (f64, f64),
        dt: f64,
    ) -> Result<Option<MemoryResult<State>>, Box<dyn Error>>
    where
        Model: OdeModel<State = State> + StateFromModelMut<State = State>,
        State: OdeState,
    {
        let x0 = problem
            .model
            .initial_state();
        self.solve_fixed(problem, x0, tspan, dt)
    }

    fn initialize_writer<Model: OdeModel<State = State>, State: OdeState>(
        &self,
        problem: &mut OdeProblem<Model, State>,
        x0: &State,
    ) -> Result<Option<WriterManager>, Box<dyn Error>> {
        // Initialize the manager for writing results to a file
        let writer = if let Some(save_folder) = &problem.save_folder {
            let mut writer_manager = WriterManager::new();
            // Initialize the manager with the user provided builders
            for event in &mut problem
                .events
                .save_events
            {
                (event.init_fn)(
                    &mut problem.model,
                    x0,
                    &mut writer_manager,
                );
            }
            // Initialize the writers from the builders
            writer_manager.initialize(save_folder)?;
            Some(writer_manager)
        } else {
            None
        };
        Ok(writer)
    }

    fn initialize_adaptive_result<State>(
        &self,
        _x0: &State,
        tspan: &(f64, f64),
        controller: &AdaptiveStepControl,
    ) -> Option<MemoryResult<State>>
    where
        State: OdeState,
    {
        // Preallocate memory for result storage if needed
        match self.save_method {
            SaveMethods::Memory => {
                let n = if let Some(max_dt) = &controller.max_dt {
                    ((tspan.1 - tspan.0) / max_dt).ceil() as usize
                } else {
                    // Default conservative allocation: 1 save per second
                    (tspan.1 - tspan.0).ceil() as usize
                };
                Some(MemoryResult::new(n))
            }
            _ => None,
        }
    }

    fn initialize_fixed_result<State>(
        &self,
        _x0: &State,
        tspan: &(f64, f64),
        controller: &FixedStepControl,
    ) -> Option<MemoryResult<State>>
    where
        State: OdeState,
    {
        // Preallocate memory for result storage if needed
        match self.save_method {
            SaveMethods::Memory => {
                let n = ((tspan.1 - tspan.0) / controller.dt).ceil() as usize;
                Some(MemoryResult::new(n))
            }
            _ => None,
        }
    }
}

#[derive(Clone, Copy)]
pub enum SolverMethods {
    Explicit(ExplicitMethods),
    Implicit,
    MultiStep,
}

impl From<ExplicitMethods> for SolverMethods {
    fn from(value: ExplicitMethods) -> Self {
        Self::Explicit(value)
    }
}

impl From<RungeKuttaMethods> for SolverMethods {
    fn from(value: RungeKuttaMethods) -> Self {
        Self::Explicit(ExplicitMethods::RungeKutta(
            value,
        ))
    }
}

impl SolverMethods {
    fn solve_adaptive<Model, State>(
        &self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut AdaptiveStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut Option<MemoryResult<State>>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState + Adaptive,
    {
        match self {
            SolverMethods::Explicit(method) => method.solve_adaptive(
                model,
                x0,
                tspan,
                controller,
                events,
                result,
                writer_manager,
            ),
            _ => todo!(),
        }
    }
    fn solve_fixed<Model, State>(
        &self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut FixedStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut Option<MemoryResult<State>>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState,
    {
        match self {
            SolverMethods::Explicit(method) => method.solve_fixed(
                model,
                x0,
                tspan,
                controller,
                events,
                result,
                writer_manager,
            ),
            _ => todo!(),
        }
    }
}

#[derive(Clone, Copy)]
pub enum ExplicitMethods {
    RungeKutta(RungeKuttaMethods),
}

impl From<RungeKuttaMethods> for ExplicitMethods {
    fn from(value: RungeKuttaMethods) -> Self {
        Self::RungeKutta(value)
    }
}

impl ExplicitMethods {
    fn solve_adaptive<Model, State>(
        &self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut AdaptiveStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut Option<MemoryResult<State>>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState + Adaptive,
    {
        match self {
            ExplicitMethods::RungeKutta(method) => method.solve_adaptive(
                model,
                x0,
                tspan,
                controller,
                events,
                result,
                writer_manager,
            ),
        }
    }

    fn solve_fixed<Model, State>(
        &self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut FixedStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut Option<MemoryResult<State>>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState,
    {
        match self {
            ExplicitMethods::RungeKutta(method) => method.solve_fixed(
                model,
                x0,
                tspan,
                controller,
                events,
                result,
                writer_manager,
            ),
        }
    }
}

/// Enum representing the available solvers supported by the framework.
#[derive(Clone, Copy)]
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
    pub fn solve_fixed<State, Model>(
        &self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut FixedStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut Option<MemoryResult<State>>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState,
    {
        match self {
            RungeKuttaMethods::DoPri45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::DORMANDPRINCE45);
                solver.solve_fixed(
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
                solver.solve_fixed(
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
                solver.solve_fixed(
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
                solver.solve_fixed(
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
                solver.solve_fixed(
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
                solver.solve_fixed(
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

    pub fn solve_adaptive<State, Model>(
        &self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut AdaptiveStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut Option<MemoryResult<State>>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState + Adaptive,
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
            RungeKuttaMethods::Rk4 => {
                panic!("classic RK4 does not support adaptive time stepping")
            }
            RungeKuttaMethods::Tsit5 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::TSITOURAS5);
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
            RungeKuttaMethods::Verner6 => {
                let mut solver = RungeKutta::new(ButcherTableau::<6, 9>::VERNER6);
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
            RungeKuttaMethods::Verner9 => {
                let mut solver = RungeKutta::new(ButcherTableau::<9, 26>::VERNER9);
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
        }
    }
}
