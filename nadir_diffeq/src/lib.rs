use std::{
    marker::PhantomData,
    ops::{AddAssign, MulAssign},
    path::PathBuf,
};

pub trait Integrable: Sized + Clone + Default + MulAssign<f64>
where
    for<'a> Self: AddAssign<&'a Self> + AddAssign<&'a Self::Derivative>,
{
    type Derivative: Clone + MulAssign<f64> + Sized + Default;
}

pub trait OdeModel<State>
where
    State: Integrable,
{
    fn f(&mut self, t: f64, state: &State, derivative: &mut State::Derivative);
}

pub trait Method<State: Integrable> {
    fn step<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        xf: &mut State,
        t: f64,
        h: f64,
    );
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

pub struct OdeSolver<Solver, State>
where
    State: Integrable,
    Solver: Method<State>,
{
    solver: Solver,
    step_method: StepMethod,
    save_method: SaveMethod,
    _phantom: PhantomData<State>,
}

impl<Solver, State> OdeSolver<Solver, State>
where
    State: Integrable,
    Solver: Method<State>,
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

        // main sim loop
        let mut i = 0;
        while t < tspan.1 {
            // save the current state and time
            if let Some(result) = &mut memory_result {
                result.insert(i, t, &x0);
            }
            match self.step_method {
                StepMethod::Fixed(dt) => {
                    self.solver.step(model, &x0, &mut xf, t, dt);
                    // update for next iteration
                    t += dt;
                    x0.clone_from(&xf);
                }
                StepMethod::Adaptive {
                    rel_tol,
                    abs_tol,
                    max_dt,
                    min_dt,
                } => {}
            }
            i += 1;
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
}

impl<State: Integrable> MemoryResult<State> {
    pub fn new(n: usize) -> Self {
        Self {
            t: vec![0.0; n],
            y: vec![State::default(); n],
        }
    }
    pub fn insert(&mut self, i: usize, t: f64, x: &State) {
        self.t[i] = t;
        self.y[i].clone_from(x);
    }
}

pub struct StageBuffer<State, const STAGES: usize>
where
    State: Integrable,
{
    pub k: [State::Derivative; STAGES],
}

pub struct ButcherTableau<const STAGES: usize> {
    pub a: [[f64; STAGES]; STAGES],
    pub b: [f64; STAGES],
    pub b2: Option<[f64; STAGES]>,
    pub c: [f64; STAGES],
}

impl ButcherTableau<4> {
    // usage is ButcherTableau::<4>::RK4
    pub const RK4: Self = Self {
        a: [
            [0., 0., 0., 0.],
            [1. / 2., 0., 0., 0.],
            [0., 1. / 2., 0., 0.],
            [0., 0., 1., 0.],
        ],
        b: [1. / 6., 1. / 3., 1. / 3., 1. / 6.],
        b2: None,
        c: [0., 1.0 / 2.0, 1.0 / 2.0, 1.0],
    };
}
impl ButcherTableau<7> {
    // usage is ButcherTableau::<7>::DORMANDRINCE45
    pub const DORMANDPRINCE45: Self = Self {
        a: [
            [0., 0., 0., 0., 0., 0., 0.],
            [1. / 5., 0., 0., 0., 0., 0., 0.],
            [3. / 40., 9. / 40., 0., 0., 0., 0., 0.],
            [44. / 45., -56. / 15., 32. / 9., 0., 0., 0., 0.],
            [
                19372. / 6561.,
                -25360. / 2187.,
                64448. / 6561.,
                -212. / 729.,
                0.,
                0.,
                0.,
            ],
            [
                9017. / 3168.,
                -355. / 33.,
                46732. / 5247.,
                49. / 176.,
                -5103. / 18656.,
                0.,
                0.,
            ],
            [
                35. / 384.,
                0.,
                500. / 1113.,
                125. / 192.,
                -2187. / 6784.,
                11. / 84.,
                0.,
            ],
        ],
        b: [
            35. / 384.,
            0.,
            500. / 1113.,
            125. / 192.,
            -2187. / 6784.,
            11. / 84.,
            0.,
        ],
        b2: Some([
            5179. / 57600.,
            0.,
            7571. / 16695.,
            393. / 640.,
            -92097. / 339200.,
            187. / 2100.,
            1. / 40.,
        ]),
        c: [0., 1. / 5., 3. / 10., 4. / 5., 8. / 9., 1.0, 1.0],
    };
}
pub struct RungeKutta<State: Integrable, const STAGES: usize> {
    stage_buffer: StageBuffer<State, STAGES>, // preallocated buffer for stage results
    calc_buffer_state: State,                 // preallocated buffer for calculations
    calc_buffer_derivative: State::Derivative,
    tableau: ButcherTableau<STAGES>,
}

impl<State: Integrable, const STAGES: usize> RungeKutta<State, STAGES> {
    pub fn new<Model>(tableau: ButcherTableau<STAGES>, _model: &Model) -> Self
    where
        Model: OdeModel<State>,
    {
        Self {
            stage_buffer: StageBuffer {
                k: std::array::from_fn(|_| State::Derivative::default()),
            },
            calc_buffer_state: State::default(),
            calc_buffer_derivative: State::Derivative::default(),
            tableau,
        }
    }
}
impl<State: Integrable, const STAGES: usize> Method<State> for RungeKutta<State, STAGES> {
    fn step<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        xf: &mut State,
        t: f64,
        h: f64,
    ) {
        let k = &mut self.stage_buffer.k;

        // k0
        model.f(t, x0, &mut k[0]);

        // k1 - ks
        for s in 1..STAGES {
            // in place calculation of intermediate points
            self.calc_buffer_state *= 0.0;
            // sum previous ks with appropriate scaling from tableau
            for i in 0..s {
                self.calc_buffer_derivative.clone_from(&k[i]);
                self.calc_buffer_derivative *= self.tableau.a[s][i];
                self.calc_buffer_state += &self.calc_buffer_derivative;
            }
            self.calc_buffer_state *= h;
            self.calc_buffer_state += x0;

            model.f(
                t + self.tableau.c[s] * h,
                &self.calc_buffer_state,
                &mut k[s],
            );
        }
        xf.clone_from(x0);
        for s in 0..STAGES {
            self.calc_buffer_derivative.clone_from(&k[s]);
            self.calc_buffer_derivative *= self.tableau.b[s] * h;
            *xf += &self.calc_buffer_derivative;
        }
    }
}
