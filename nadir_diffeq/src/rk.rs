use std::array;

use tolerance::Tolerance;

use crate::{
    Integrable, OdeModel, StepMethod,
    events::Events,
    saving::ResultStorage,
    stepping::{FixedStepControl, StepPIDControl},
    tableau::ButcherTableau,
};

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
        step_method: &mut StepMethod,
        events: &mut Vec<Events<Model, State>>,
        result: &mut ResultStorage<State>,
    ) {
        match step_method {
            StepMethod::Fixed(controller) => {
                controller.next_time = tspan.0;
                self.solve_fixed(model, x0, tspan, controller, events, result)
            }
            StepMethod::Adaptive(controller) => {
                self.solve_adaptive(model, x0, tspan, controller, events, result)
            }
        }
    }

    pub fn solve_fixed<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut FixedStepControl,
        events: &mut Vec<Events<Model, State>>,
        result: &mut ResultStorage<State>,
    ) {
        let mut t = tspan.0;
        let mut function_calls = 0;
        self.x.clone_from(x0);
        while t < tspan.1 {
            self.step(model, t, controller.dt, false, &mut function_calls);
            // determine if any events occurred
            // todo
            result.save(t, &self.y);
            // determine if any events will occur
            t += controller.dt;
            // todo
            // for event in events {
            //     match event {
            //         Events::Periodic(e) => {
            //             if t > e.next_time {
            //                 controller.next_time = e.next_time;
            //             }
            //         }
            //     }
            // }

            // initialize next loop
            self.x.clone_from(&self.y);
        }
    }

    pub fn solve_adaptive<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut StepPIDControl,
        events: &mut Vec<Events<Model, State>>,
        result: &mut ResultStorage<State>,
    ) {
        let mut t = tspan.0;
        self.x.clone_from(x0);
        let mut dt = 1e-3; // initial dt

        let mut accept_counter = 0;
        let mut reject_counter = 0;
        let mut function_calls = 0;
        while t < tspan.1 {
            // println!("{t} {dt}");
            // Ensure we don't step past the end
            if t + dt > tspan.1 {
                dt = tspan.1 - t;
            }

            // Trial step
            self.step(model, t, dt, true, &mut function_calls);

            // Calculate error
            let error = self.compute_error(
                &self.y,
                &self.y_star,
                controller.rel_tol,
                controller.abs_tol,
            );

            // Calculate new step size based on dt
            let new_dt = controller.step(dt, error);

            // Check if step is accepted
            if error <= 1.0 {
                // Step ACCEPTED: advance time, save result, and update state
                t += dt;
                result.save(t, &self.y);
                self.x.clone_from(&self.y);
                dt = new_dt;

                accept_counter += 1;
            } else {
                // Step REJECTED: try again with reduced step size
                dt = new_dt;

                // Safety check for minimum step size
                if let Some(min) = controller.min_dt {
                    if dt <= min {
                        // If we've hit minimum step size but error is still too large,
                        // we might need to accept the step anyway or signal an error
                        panic!("Minimum step size reached but error is still too large");
                    }
                }

                reject_counter += 1;
            }
            // Add a constant minimum step size regardless of min_dt parameter
            const EMERGENCY_MIN_DT: f64 = 1e-10;

            if dt < EMERGENCY_MIN_DT {
                panic!(
                    "Emergency minimum step size reached at t = {}, error = {}",
                    t, error
                );
            }
        }

        println!(
            "Adaptive step size: accepted {} steps, rejected {} steps, function calls: {}",
            accept_counter, reject_counter, function_calls
        );
    }

    pub fn step<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        t: f64,
        h: f64,
        adaptive: bool,
        function_calls: &mut i32,
    ) {
        let k = &mut self.buffers.stage.k;

        // k0
        model.f(t, &self.x, &mut k[0]);
        *function_calls += 1;

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
            *function_calls += 1;
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
