use std::{
    marker::PhantomData,
    ops::{AddAssign, MulAssign},
    path::PathBuf,
};
pub mod result;
pub mod rk;
pub mod state_array;
pub mod tableau;
use crate::rk::RungeKutta;
use crate::tableau::ButcherTableau;
use result::{MemoryResult, ResultStorage};
use tolerance::Tolerance;

pub trait Integrable: Sized + Clone + Default + MulAssign<f64>
where
    for<'a> Self: AddAssign<&'a Self> + AddAssign<&'a Self::Derivative>,
{
    type Derivative: Clone + MulAssign<f64> + Sized + Default;
    type Tolerance: Tolerance<State = Self>;
}

pub trait OdeModel<State>
where
    State: Integrable,
{
    fn f(&mut self, t: f64, state: &State, derivative: &mut State::Derivative);
}

#[derive(Copy, Clone)]
pub enum StepMethod {
    Fixed(f64),
    Adaptive {
        rel_tol: f64,
        abs_tol: f64,
        max_dt: Option<f64>,
        min_dt: Option<f64>,
    },
}

pub enum SaveMethod {
    Memory,
    File(PathBuf),
    None, // no saving by the solver, saving should be handled by the Model
}

pub enum Solver {
    DoPri45,
    Rk4,
    Tsit5,
}

pub struct OdeProblem<State>
where
    State: Integrable,
{
    solver: Solver,
    step_method: StepMethod,
    save_method: SaveMethod,
    _phantom: PhantomData<State>,
}

impl<State> OdeProblem<State>
where
    State: Integrable,
{
    pub fn new(solver: Solver, step_method: StepMethod, save_method: SaveMethod) -> Self {
        Self {
            solver,
            save_method,
            step_method,
            _phantom: PhantomData,
        }
    }

    pub fn solve<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
    ) -> ResultStorage<State> {
        // preallocate the memory result if there is one
        let mut result = match self.save_method {
            SaveMethod::Memory => {
                let n = match self.step_method {
                    StepMethod::Fixed(dt) => ((tspan.1 - tspan.0) / dt).ceil() as usize,
                    StepMethod::Adaptive {
                        rel_tol: _,
                        abs_tol: _,
                        max_dt,
                        min_dt: _,
                    } => {
                        if let Some(max_dt) = &max_dt {
                            ((tspan.1 - tspan.0) / max_dt).ceil() as usize
                        } else {
                            // just preallocate 1 per second for now?
                            (tspan.1 - tspan.0).ceil() as usize
                        }
                    }
                };
                ResultStorage::Memory(MemoryResult::<State>::new(n))
            }
            _ => ResultStorage::None,
        };

        match self.solver {
            Solver::DoPri45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<7>::DORMANDPRINCE45);
                solver.solve(model, x0, tspan, self.step_method, &mut result);
            }
            Solver::Rk4 => {
                let mut solver = RungeKutta::new(ButcherTableau::<4>::RK4);
                solver.solve(model, x0, tspan, self.step_method, &mut result);
            }
            Solver::Tsit5 => {
                let mut solver = RungeKutta::new(ButcherTableau::<7>::TSITOURAS5);
                solver.solve(model, x0, tspan, self.step_method, &mut result);
            }
        }
        result.truncate();
        result
    }
}
