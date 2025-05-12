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

    pub fn with_tolerances(mut self, tol: State::Tolerance) -> Self {
        self.tolerances = tol;
        self
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
            } => self.solve_adaptive(model, x0, tspan, rel_tol, abs_tol, max_dt, min_dt, result),
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

    pub fn solve_adaptive<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        rel_tol: f64,
        abs_tol: f64,
        max_dt: Option<f64>,
        min_dt: Option<f64>,
        result: &mut ResultStorage<State>,
    ) {
        let mut t = tspan.0;
        self.x.clone_from(x0);
        let mut dt = 1e-3; // initial dt
        while t < tspan.1 {
            // Ensure we don't step past the end
            if t + dt > tspan.1 {
                dt = tspan.1 - t;
            }

            // Trial step
            self.step(model, t, dt, true);

            // Calculate error
            let error = self.compute_error(&self.y, &self.y_star, rel_tol, abs_tol);

            // Calculate new step size based on dt
            let new_dt = compute_adaptive_step(dt, error, self.tableau.order);

            // Apply min/max step size constraints
            let new_dt = match (min_dt, max_dt) {
                (Some(min), Some(max)) => new_dt.min(max).max(min),
                (Some(min), None) => new_dt.max(min),
                (None, Some(max)) => new_dt.min(max),
                (None, None) => new_dt,
            };

            // Check if step is accepted
            if error <= 1.0 {
                // Step ACCEPTED: advance time, save result, and update state
                t += dt;
                result.save(t, &self.y);
                self.x.clone_from(&self.y);
                dt = new_dt;
            } else {
                // Step REJECTED: try again with reduced step size
                dt = new_dt;

                // Safety check for minimum step size
                if let Some(min) = min_dt {
                    if dt <= min {
                        // If we've hit minimum step size but error is still too large,
                        // we might need to accept the step anyway or signal an error
                        println!("Warning: Minimum step size reached but error is still too large");
                        t += dt;
                        result.save(t, &self.y);
                        self.x.clone_from(&self.y);
                    }
                }
            }
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

    pub fn compute_error(&self, y: &State, y_star: &State, rel_tol: f64, abs_tol: f64) -> f64 {
        self.tolerances.compute_error(y, y_star, rel_tol, abs_tol)
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

pub fn compute_adaptive_step(h: f64, error: f64, order: usize) -> f64 {
    const MAX_FACTOR: f64 = 5.0; // Don't increase step by more than 5x
    const MIN_FACTOR: f64 = 0.1; // Don't decrease step by more than 10x
    const SAFETY_FACTOR: f64 = 0.9;

    // Handle zero or very small error
    if error < 1e-15 {
        return h * MAX_FACTOR;
    }

    // Standard step size control formula
    let exponent = 1.0 / (order as f64);
    let raw_factor = SAFETY_FACTOR * (1.0 / error).powf(exponent);

    let limited_factor = raw_factor.min(MAX_FACTOR).max(MIN_FACTOR);

    // Return the new step size
    h * limited_factor
}
