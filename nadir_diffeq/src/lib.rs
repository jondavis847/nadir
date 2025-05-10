use std::{
    marker::PhantomData,
    ops::{AddAssign, MulAssign},
    path::PathBuf,
};
pub mod rk;
pub mod state_array;
pub mod tableau;
use crate::rk::RungeKutta;
use crate::tableau::ButcherTableau;
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

pub struct OdeSolver<State>
where
    State: Integrable,
{
    solver: Solver,
    step_method: StepMethod,
    save_method: SaveMethod,
    _phantom: PhantomData<State>,
}

pub enum Solver {
    DoPri45,
    Rk4,
}

impl Solver {
    fn to_storage<State>(&self) -> SolverStorage<State>
    where
        State: Integrable,
    {
        match self {
            Solver::DoPri45 => SolverStorage::<State>::DoPri45(RungeKutta::new(
                ButcherTableau::<7>::DORMANDPRINCE45,
            )),
            Solver::Rk4 => SolverStorage::<State>::Rk4(RungeKutta::new(ButcherTableau::<4>::RK4)),
        }
    }
}

enum SolverStorage<State>
where
    State: Integrable,
{
    DoPri45(RungeKutta<State, 7>),
    Rk4(RungeKutta<State, 4>),
}

impl<State> SolverStorage<State>
where
    State: Integrable,
{
    fn step<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        xf: &mut State,
        t: f64,
        h: f64,
    ) {
        match self {
            SolverStorage::DoPri45(rk) => rk.step(model, x0, xf, t, h),
            SolverStorage::Rk4(rk) => rk.step(model, x0, xf, t, h),
        }
    }

    fn check_error(&mut self, x0: &State, xf: &State, rel_tol: f64, abs_tol: f64) -> bool {
        match self {
            SolverStorage::DoPri45(rk) => rk.check_error(x0, xf, rel_tol, abs_tol),
            SolverStorage::Rk4(rk) => rk.check_error(x0, xf, rel_tol, abs_tol),
        }
    }
}

impl<State> OdeSolver<State>
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
    ) -> Option<MemoryResult<State>> {
        // initialize
        let mut x0 = x0.clone();
        let mut xf = State::default();
        let mut t = tspan.0;

        let mut solver = self.solver.to_storage();

        // preallocate the memory result if there is one
        let mut memory_result = match self.save_method {
            SaveMethod::Memory => {
                let n = match self.step_method {
                    StepMethod::Fixed(dt) => ((tspan.1 - tspan.0) / dt).ceil() as usize,
                    StepMethod::Adaptive {
                        rel_tol,
                        abs_tol,
                        max_dt,
                        min_dt,
                    } => {
                        if let Some(max_dt) = &max_dt {
                            ((tspan.1 - tspan.0) / max_dt).ceil() as usize
                        } else {
                            // just preallocate 1 per second for now?
                            (tspan.1 - tspan.0).ceil() as usize
                        }
                    }
                };
                Some(MemoryResult::<State>::new(n))
            }
            _ => None,
        };

        // adaptive step variables
        let mut var_dt = 1e-3;

        // main sim loop
        let mut i = 0;

        while t < tspan.1 {
            // save the current state and time
            if let Some(result) = &mut memory_result {
                if i >= result.len() {
                    result.extend()
                }
                result.insert(i, t, &x0);
            }
            match self.step_method {
                StepMethod::Fixed(dt) => {
                    solver.step(model, &x0, &mut xf, t, dt);
                    // update for next iteration
                    t += dt;
                    x0.clone_from(&xf);
                }
                StepMethod::Adaptive {
                    rel_tol,
                    abs_tol,
                    max_dt,
                    min_dt,
                } => {
                    if let Some(min_dt) = &min_dt {
                        var_dt = min_dt.max(var_dt);
                    }
                    if let Some(max_dt) = &max_dt {
                        var_dt = max_dt.min(var_dt);
                    }

                    solver.step(model, &x0, &mut xf, t, var_dt);
                    solver.check_error(&x0, &xf, rel_tol, abs_tol);
                }
            }
            i += 1;
        }
        if let Some(result) = &mut memory_result {
            result.truncate();
        }
        memory_result
    }
}

pub struct MemoryResult<State>
where
    State: Integrable,
{
    pub t: Vec<f64>,
    pub y: Vec<State>,
    n: usize, // manual tracking of adaptive elements
}

impl<State: Integrable> MemoryResult<State> {
    pub fn new(n: usize) -> Self {
        Self {
            t: vec![0.0; n],
            y: vec![State::default(); n],
            n: 0,
        }
    }
    fn insert(&mut self, i: usize, t: f64, x: &State) {
        self.t[i] = t;
        self.y[i].clone_from(x);
        self.n = i;
    }

    fn len(&self) -> usize {
        self.t.len()
    }

    // doubles the length if capapcity is reached
    fn extend(&mut self) {
        self.t.extend(vec![0.0; self.len()]);
        self.y.extend(vec![State::default(); self.len()]);
    }

    fn truncate(&mut self) {
        self.t.truncate(self.n);
        self.y.truncate(self.n);
    }
}
