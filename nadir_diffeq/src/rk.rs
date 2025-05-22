use std::array;

use tolerance::Tolerance;

use crate::{
    Integrable, OdeModel, StepMethod,
    events::EventManager,
    saving::ResultStorage,
    stepping::{AdaptiveStepControl, FixedStepControl},
    tableau::ButcherTableau,
};

// preallocated buffers for intermediate calculations
#[derive(Default)]
struct RKBuffers<State: Integrable, const STAGES: usize> {
    stage: StageBuffer<State, STAGES>,
    state: State,
    derivative: State::Derivative,
    interpolant: State,
}

pub struct RungeKutta<State: Integrable, const ORDER: usize, const STAGES: usize> {
    x: State,
    y: State,
    y_tilde: State,
    tableau: ButcherTableau<ORDER, STAGES>,
    tolerances: State::Tolerance,
    buffers: RKBuffers<State, STAGES>,
    first_step: bool,
}

impl<State: Integrable, const ORDER: usize, const STAGES: usize> RungeKutta<State, ORDER, STAGES> {
    pub fn new(tableau: ButcherTableau<ORDER, STAGES>) -> Self
    where
        State: Integrable,
    {
        Self {
            buffers: RKBuffers::default(),
            x: State::default(),
            y: State::default(),
            y_tilde: State::default(),
            tableau,
            tolerances: State::Tolerance::default(),
            first_step: true,
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
        events: &mut EventManager<Model, State>,
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
        events: &mut EventManager<Model, State>,
        result: &mut ResultStorage<State>,
    ) {
        // Counter to count number of function calls
        let mut function_calls = 0;

        let mut t = tspan.0;

        // Copy initial state
        self.x.clone_from(x0);

        // Save the true initial state before any processing
        result.save(t, &self.x);

        // Process initial events if any are scheduled at t0
        if events.process_events(model, &mut self.x, t) {
            // If initial events changed state, save the updated state
            result.save(t, &self.x);
        };

        while t < tspan.1 {
            // Determine step size - standard dt or adjusted for upcoming event
            let next_event_time = events.next_time();
            let mut dt = if next_event_time > t && next_event_time < t + controller.dt {
                // Adjust step size to land exactly on the event
                next_event_time - t
            } else {
                controller.dt
            };

            // Ensure we don't step past the end
            if t + dt > tspan.1 {
                dt = tspan.1 - t;
            }

            // Take a step
            self.step(model, t, dt, false, &mut function_calls);

            // Update time based on dt
            t += dt;

            // Save the result for this time step
            result.save(t, &self.y);

            // Run any events
            if events.process_events(model, &mut self.y, t) {
                // Save the result after events
                result.save(t, &self.y);
            };

            // Initialize next loop
            self.x.clone_from(&self.y);

            if self.tableau.fsal {
                // Reuse last stage from previous step as first stage
                let (k0, ks) = self.buffers.stage.k.split_at_mut(1);
                k0[0].clone_from(ks.last().unwrap());
            }
        }
    }

    pub fn solve_adaptive<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut AdaptiveStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut ResultStorage<State>,
    ) {
        let mut accept_counter = 0;
        let mut reject_counter = 0;
        let mut function_calls = 0;

        let mut t = tspan.0;
        self.x.clone_from(x0);
        let mut dt = 1e-3; // initial dt

        // Save the true initial state before any processing
        result.save(t, &self.x);

        // Process initial events if any are scheduled at t0
        if events.process_events(model, &mut self.x, t) {
            // If initial events changed state, save the updated state
            result.save(t, &self.x);
        };

        while t < tspan.1 {
            // Determine step size - standard dt or adjusted for upcoming event
            let next_event_time = events.next_time();
            if next_event_time > t && next_event_time < t + dt {
                // Adjust step size to land exactly on the event
                dt = next_event_time - t
            };

            // Ensure we don't step past the end
            if t + dt > tspan.1 {
                dt = tspan.1 - t;
            }

            // Trial step
            self.step(model, t, dt, true, &mut function_calls);

            // Calculate error
            let error = self.compute_error(
                &self.y,
                &self.x,
                &self.y_tilde,
                controller.rel_tol,
                controller.abs_tol,
            );

            // Calculate new step size based on dt
            let new_dt = controller.step(dt, error, ORDER);

            // Check if step is accepted
            if error <= 1.0 {
                // Step ACCEPTED: advance time, save result, and update state

                t += dt;
                // Save the true state before any event processing
                result.save(t, &self.y);
                // Process events if any occurred
                if events.process_events(model, &mut self.y, t) {
                    // Events changed state, save the updated state
                    result.save(t, &self.y);
                };

                self.x.clone_from(&self.y);
                dt = new_dt;

                accept_counter += 1;

                if self.tableau.fsal {
                    // Reuse last stage from previous step as first stage
                    let (k0, ks) = self.buffers.stage.k.split_at_mut(1);
                    k0[0].clone_from(ks.last().unwrap());
                }
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

        if self.tableau.fsal {
            // FSAL method implementation
            if self.first_step {
                model.f(t, &self.x, &mut k[0]);
                *function_calls += 1;
                self.first_step = false;
            } // else k0 set up a function if step size was accepted

            // Compute intermediate stages k1 through k[STAGES-2]
            for s in 1..STAGES - 1 {
                self.buffers.state *= 0.0;
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

            // Calculate solution using stages 0 through STAGES-2
            self.y.clone_from(&self.x);
            for s in 0..STAGES - 1 {
                self.buffers.derivative.clone_from(&k[s]);
                self.buffers.derivative *= self.tableau.b[s] * h;
                self.y += &self.buffers.derivative;
            }

            // Calculate final stage at new solution point
            model.f(t + h, &self.y, &mut k[STAGES - 1]);
            *function_calls += 1;
        } else {
            // Standard (non-FSAL) method implementation
            model.f(t, &self.x, &mut k[0]);
            *function_calls += 1;

            for s in 1..STAGES {
                self.buffers.state *= 0.0;
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
        }

        // Adaptive error estimation - same for both methods
        if adaptive && self.tableau.b_tilde.is_some() {
            let b_tilde = self.tableau.b_tilde.unwrap();
            self.y_tilde *= 0.0; //reset
            for s in 0..STAGES {
                self.buffers.derivative.clone_from(&k[s]);
                self.buffers.derivative *= b_tilde[s];
                self.y_tilde += &self.buffers.derivative;
            }
            self.y_tilde *= h;
        }
    }

    pub fn compute_error(
        &self,
        y: &State,
        y_prev: &State,
        y_tilde: &State,
        rel_tol: f64,
        abs_tol: f64,
    ) -> f64 {
        self.tolerances
            .compute_error(y, y_prev, y_tilde, rel_tol, abs_tol)
    }

    pub fn interpolate(&mut self, t0: f64, dt: f64, t: f64) {
        if let Some(bi) = &self.tableau.bi {
            if t < t0 || t > t0 + dt {
                panic!("t out of range for interpolation - todo extrapolation?")
            }

            // reset buffers
            self.buffers.interpolant *= 0.0;

            // calculate theta
            let theta = (t - t0) / dt;

            for s in 0..STAGES {
                let mut b = 0.0;
                for i in 0..ORDER - 1 {
                    b += bi[s][i] * theta.powi(i as i32 + 1);
                }

                self.buffers.derivative.clone_from(&self.buffers.stage.k[s]);
                self.buffers.derivative *= b;
                self.buffers.interpolant += &self.buffers.derivative;
            }

            // store answer in interpolant buffer, must be retrieved for usage outside this function
            self.buffers.interpolant *= dt;
            self.buffers.interpolant += &self.x;
        } else {
            panic!("No interpolation coefficients for solver")
        }
    }
}

#[derive(Debug, Clone)]
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
