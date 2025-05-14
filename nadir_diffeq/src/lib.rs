use std::{
    fs::File,
    io::BufWriter,
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
use csv::Writer;
use result::{MemoryResult, ResultStorage};
use tolerance::Tolerance;

pub trait Integrable: Sized + Clone + Default + MulAssign<f64>
where
    for<'a> Self: AddAssign<&'a Self> + AddAssign<&'a Self::Derivative>,
{
    type Derivative: Clone + MulAssign<f64> + Sized + Default;
    type Tolerance: Tolerance<State = Self>;

    fn initialize_writer(_path: &PathBuf) -> Option<Writer<BufWriter<File>>> {
        None
    }

    fn save_to_writer(&self, _writer: &mut Writer<BufWriter<File>>, _t: f64) {}
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
    Adaptive(StepPIDControl),
}

pub enum SaveMethod {
    Memory,
    File(PathBuf),
    None, // no saving by the solver, saving should be handled by the Model
}

pub enum Solver {
    DoPri45,
    New45,
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
        match (&solver, &step_method) {
            (Solver::Rk4, StepMethod::Adaptive(_)) => {
                panic!("Rk4 solver does not support adaptive step size")
            }
            _ => {}
        }

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
        let mut result = match &self.save_method {
            SaveMethod::Memory => {
                let n = match self.step_method {
                    StepMethod::Fixed(dt) => ((tspan.1 - tspan.0) / dt).ceil() as usize,
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
                let mut solver = RungeKutta::new(ButcherTableau::<7>::DORMANDPRINCE45);
                solver.solve(model, x0, tspan, self.step_method, &mut result);
            }
            Solver::New45 => {
                let mut solver = RungeKutta::new(ButcherTableau::<7>::NEW45);
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

#[derive(Clone, Copy)]
pub struct StepPIDControl {
    rel_tol: f64,
    abs_tol: f64,
    kp: f64,
    ki: f64,
    kd: f64,
    min_dt: Option<f64>,
    max_dt: Option<f64>,
    min_growth: Option<f64>,
    max_growth: Option<f64>,
    err_now: f64,
    err_prev: f64,
    err_prevprev: f64,
}

impl Default for StepPIDControl {
    fn default() -> Self {
        Self {
            rel_tol: 1e-3,
            abs_tol: 1e-6,
            kp: 0.075,
            ki: 0.01,
            kd: 0.175,
            min_dt: None,
            max_dt: None,
            min_growth: Some(0.1),
            max_growth: Some(5.0),
            err_now: 1.0, //1.0 ensures small initial steps and no divide by 0
            err_prev: 1.0,
            err_prevprev: 1.0,
        }
    }
}

impl StepPIDControl {
    pub fn new(
        rel_tol: f64,
        abs_tol: f64,
        kp: f64,
        ki: f64,
        kd: f64,
        min_dt: Option<f64>,
        max_dt: Option<f64>,
        min_growth: Option<f64>,
        max_growth: Option<f64>,
    ) -> Self {
        Self {
            rel_tol,
            abs_tol,
            kp,
            ki,
            kd,
            min_dt,
            max_dt,
            min_growth,
            max_growth,
            err_now: 1.0, //1.0 ensures small initial steps and no divide by 0
            err_prev: 1.0,
            err_prevprev: 1.0,
        }
    }

    pub fn with_tolerances(mut self, rel_tol: f64, abs_tol: f64) -> Self {
        self.rel_tol = rel_tol;
        self.abs_tol = abs_tol;
        self
    }

    pub fn step(&mut self, h: f64, err_now: f64) -> f64 {
        self.err_prevprev = self.err_prev;
        self.err_prev = self.err_now;
        self.err_now = err_now;

        // divide by 0 protection
        const EPS: f64 = 1e-14;
        let e0 = self.err_now.max(EPS);
        let e1 = self.err_prev.max(EPS);
        let e2 = self.err_prevprev.max(EPS);

        // calculate the growth factor
        let mut factor = e0.powf(-self.kp) * (e0 / e1).powf(-self.kd) * (e1 / e2).powf(self.ki);

        // limit the growth of the step size
        if let Some(min_growth) = self.min_growth {
            factor = factor.max(min_growth);
        }
        if let Some(max_growth) = self.max_growth {
            factor = factor.min(max_growth);
        }

        let mut new_dt = h * factor;
        // limit the step size
        if let Some(min_dt) = self.min_dt {
            new_dt = new_dt.max(min_dt);
        }
        if let Some(max_dt) = self.max_dt {
            new_dt = new_dt.min(max_dt);
        }
        new_dt
    }
}
