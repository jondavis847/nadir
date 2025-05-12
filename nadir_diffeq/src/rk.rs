use std::array;

use tolerance::Tolerance;

use crate::{Integrable, OdeModel, StepMethod, result::ResultStorage, tableau::ButcherTableau};

// preallocated buffers for intermediate calculations
#[derive(Default)]
struct RKBuffers<State: Integrable, const STAGES: usize> {
    stage: StageBuffer<State, STAGES>,
    state: State,
    derivative: State::Derivative,
}

pub struct RungeKutta<State: Integrable, const STAGES: usize> {
    x: State,
    y: State,
    y_star: State,
    tableau: ButcherTableau<STAGES>,
    tolerances: State::Tolerance,
    buffers: RKBuffers<State, STAGES>,
}

impl<State: Integrable, const STAGES: usize> RungeKutta<State, STAGES> {
    pub fn new(tableau: ButcherTableau<STAGES>) -> Self
    where
        State: Integrable,
    {
        Self {
            buffers: RKBuffers::default(),
            x: State::default(),
            y: State::default(),
            y_star: State::default(),
            tableau,
            tolerances: State::Tolerance::default(),
        }
    }

    pub fn solve<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        step_method: StepMethod,
        result: &mut ResultStorage<State>,
    ) {
        match step_method {
            StepMethod::Fixed(dt) => self.solve_fixed(model, x0, tspan, dt, result),
            StepMethod::Adaptive {
                rel_tol,
                abs_tol,
                max_dt,
                min_dt,
            } => {} // todo ! self.solve_adaptive(model, x0, tspan, rel_tol, abs_tol, max_dt, min_dt, result),
        }
    }

    pub fn solve_fixed<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        dt: f64,
        result: &mut ResultStorage<State>,
    ) {
        let mut t = tspan.0;
        self.x.clone_from(x0);
        while t < tspan.1 {
            self.step(model, t, dt, false);
            result.save(t, &self.y);
            t += dt;
            self.x.clone_from(&self.y);
        }
    }

    pub fn step<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        t: f64,
        h: f64,
        adaptive: bool,
    ) {
        let k = &mut self.buffers.stage.k;

        // k0
        model.f(t, &self.x, &mut k[0]);

        // k1 - ks
        for s in 1..STAGES {
            // in place calculation of intermediate points
            self.buffers.state *= 0.0;
            // sum previous ks with appropriate scaling from tableau
            for i in 0..s {
                self.buffers.derivative.clone_from(&k[i]);
                self.buffers.derivative *= self.tableau.a[s][i];
                self.buffers.state += &self.buffers.derivative;
            }
            self.buffers.state *= h;
            self.buffers.state += &self.x;

            model.f(t + self.tableau.c[s] * h, &self.buffers.state, &mut k[s]);
        }
        self.y.clone_from(&self.x);
        for s in 0..STAGES {
            self.buffers.derivative.clone_from(&k[s]);
            self.buffers.derivative *= self.tableau.b[s] * h;
            self.y += &self.buffers.derivative;
        }

        // adaptive solving logic
        if adaptive {
            if let Some(b_star) = self.tableau.b_star {
                self.y_star.clone_from(&self.x);
                for s in 0..STAGES {
                    self.buffers.derivative.clone_from(&k[s]);
                    self.buffers.derivative *= b_star[s] * h;
                    self.y_star += &self.buffers.derivative;
                }
            }
        }
    }

    pub fn check_error(&mut self, x0: &State, xf: &State, rel_tol: f64, abs_tol: f64) -> bool {
        self.tolerances.check_error(x0, xf, rel_tol, abs_tol)
    }
}

pub struct StageBuffer<State, const STAGES: usize>
where
    State: Integrable,
{
    pub k: [State::Derivative; STAGES],
}

impl<State, const STAGES: usize> Default for StageBuffer<State, STAGES>
where
    State: Integrable,
{
    fn default() -> Self {
        Self {
            k: array::from_fn(|_| State::Derivative::default()),
        }
    }
}
