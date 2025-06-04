use std::error::Error;

use crate::{
    OdeModel,
    events::EventManager,
    rk::RungeKutta,
    saving::{ResultStorage, WriterManager},
    state::{Adaptive, OdeState},
    stepping::{AdaptiveStepControl, FixedStepControl},
    tableau::ButcherTableau,
};

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

impl Solver {
    pub fn solve_adaptive<State, Model>(
        &self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut AdaptiveStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut ResultStorage<State>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState + Adaptive,
    {
        match self {
            Solver::DoPri45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::DORMANDPRINCE45);
                solver.solve_adaptive(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::New45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::NEW45);
                solver.solve_adaptive(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::Rk4 => {
                let mut solver = RungeKutta::new(ButcherTableau::<4, 4>::RK4);
                solver.solve_adaptive(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::Tsit5 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::TSITOURAS5);
                solver.solve_adaptive(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::Verner6 => {
                let mut solver = RungeKutta::new(ButcherTableau::<6, 9>::VERNER6);
                solver.solve_adaptive(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::Verner9 => {
                let mut solver = RungeKutta::new(ButcherTableau::<9, 26>::VERNER9);
                solver.solve_adaptive(model, x0, tspan, controller, events, result, writer_manager)
            }
        }
    }

    pub fn solve_fixed<State, Model>(
        &self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut FixedStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut ResultStorage<State>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState,
    {
        match self {
            Solver::DoPri45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::DORMANDPRINCE45);
                solver.solve_fixed(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::New45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::NEW45);
                solver.solve_fixed(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::Rk4 => {
                let mut solver = RungeKutta::new(ButcherTableau::<4, 4>::RK4);
                solver.solve_fixed(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::Tsit5 => {
                let mut solver = RungeKutta::new(ButcherTableau::<5, 7>::TSITOURAS5);
                solver.solve_fixed(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::Verner6 => {
                let mut solver = RungeKutta::new(ButcherTableau::<6, 9>::VERNER6);
                solver.solve_fixed(model, x0, tspan, controller, events, result, writer_manager)
            }
            Solver::Verner9 => {
                let mut solver = RungeKutta::new(ButcherTableau::<9, 26>::VERNER9);
                solver.solve_fixed(model, x0, tspan, controller, events, result, writer_manager)
            }
        }
    }
}
