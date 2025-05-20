use std::{
    fmt::Debug,
    fs::File,
    io::BufWriter,
    ops::{AddAssign, MulAssign},
    path::PathBuf,
};
pub mod events;
pub mod rk;
pub mod saving;
pub mod state_array;
pub mod stepping;
pub mod tableau;
use crate::rk::RungeKutta;
use crate::tableau::ButcherTableau;
use csv::Writer;
use events::{EventManager, PeriodicEvent};
use saving::{MemoryResult, ResultStorage, SaveMethod};
use stepping::StepMethod;
use tolerance::Tolerance;

pub trait Integrable: Sized + Clone + Default + MulAssign<f64> + Debug
where
    for<'a> Self: AddAssign<&'a Self> + AddAssign<&'a Self::Derivative>,
{
    type Derivative: Clone + MulAssign<f64> + Sized + Default + Debug;
    type Tolerance: Tolerance<State = Self>;

    fn initialize_writer(_path: &PathBuf) -> Option<Writer<BufWriter<File>>> {
        None
    }

    fn save_to_writer(&self, _writer: &mut Writer<BufWriter<File>>, _t: f64) {}
}

pub trait OdeModel<State>: Debug
where
    State: Integrable,
{
    fn f(&mut self, t: f64, state: &State, derivative: &mut State::Derivative);
}

pub enum Solver {
    DoPri45,
    New45,
    Rk4,
    Tsit5,
    Verner6,
    Verner9,
}

pub struct OdeProblem<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable,
{
    model: Model,
    solver: Solver,
    step_method: StepMethod,
    save_method: SaveMethod,
    events: EventManager<Model, State>,
}

impl<Model, State> OdeProblem<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable,
{
    pub fn new(
        model: Model,
        solver: Solver,
        step_method: StepMethod,
        save_method: SaveMethod,
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

    pub fn with_event_periodic(mut self, event: PeriodicEvent<Model, State>) -> Self {
        self.events.add_periodic(event);
        self
    }

    pub fn solve(&mut self, x0: &State, tspan: (f64, f64)) -> ResultStorage<State> {
        // preallocate the memory result if there is one
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
                            // just preallocate 1 per second for now?
                            (tspan.1 - tspan.0).ceil() as usize
                        }
                    }
                };
                ResultStorage::Memory(MemoryResult::<State>::new(n))
            }
            SaveMethod::File(path) => {
                if let Some(writer) = State::initialize_writer(path) {
                    ResultStorage::File(writer)
                } else {
                    ResultStorage::None
                }
            }
            _ => ResultStorage::None,
        };

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
        result.truncate();
        result
    }
}
