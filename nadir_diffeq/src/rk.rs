use std::{array, error::Error, f64::INFINITY, mem::swap};

use crate::{
    OdeModel,
    events::{ContinuousEvent, EventManager},
    saving::{MemoryResult, WriterManager},
    state::{Adaptive, OdeState},
    stepping::{AdaptiveStepControl, FixedStepControl},
    tableau::ButcherTableau,
};

/// A reusable buffer holding all intermediate data required for a Runge-Kutta integration step.
///
/// This includes buffers for each stage derivative `k`, an intermediate state, a derivative buffer,
/// and an interpolated state used for dense output or event location.
#[derive(Debug)]
struct RKBuffers<State: OdeState, const STAGES: usize> {
    stage: StageBuffer<State, STAGES>,
    state: State,
    derivative: State,
    interpolant: State,
}

impl<State: OdeState, const STAGES: usize> RKBuffers<State, STAGES> {
    fn new() -> Self {
        Self {
            stage: StageBuffer::new(),
            state: State::default(),
            derivative: State::default(),
            interpolant: State::default(),
        }
    }

    pub fn init(&mut self, x0: &State) {
        self.state.clone_from(x0);
        self.derivative.clone_from(x0);
        self.interpolant.clone_from(x0);
        for buffer in &mut self.stage.k {
            buffer.clone_from(x0);
        }
    }
}

/// A generic Runge-Kutta solver capable of fixed or adaptive integration with support for:
/// - Embedded error estimation
/// - First-same-as-last (FSAL) optimizations
/// - Dense output via interpolation
/// - Periodic and continuous event handling
///
/// # Type Parameters
/// - `ORDER`: The order of the method (e.g., 5 for Dormand-Prince 4(5))
/// - `STAGES`: Number of stages in the Butcher tableau
#[derive(Debug)]
pub struct RungeKutta<State: OdeState, const ORDER: usize, const STAGES: usize> {
    x: State,
    y: State,
    y_tilde: State,
    tableau: ButcherTableau<ORDER, STAGES>,
    buffers: RKBuffers<State, STAGES>,
    first_step: bool,
}

impl<State: OdeState, const ORDER: usize, const STAGES: usize> RungeKutta<State, ORDER, STAGES> {
    /// Constructs a new Runge-Kutta solver using a specific Butcher tableau.

    pub fn new(tableau: ButcherTableau<ORDER, STAGES>) -> Self {
        Self {
            buffers: RKBuffers::new(),
            x: State::default(),
            y: State::default(),
            y_tilde: State::default(),
            tableau,
            first_step: true,
        }
    }

    /// Solves the system using fixed step size control, handling periodic events.
    pub fn solve_fixed<Model: OdeModel<State = State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut FixedStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut Option<MemoryResult<State>>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>> {
        // Counter to count number of function calls
        let mut function_calls = 0;
        let mut t = tspan.0;

        // Copy initial state to all buffers to make sure length matches initial size of dynamically sized State
        self.x.clone_from(x0);
        self.y.clone_from(x0);
        self.y_tilde.clone_from(x0);
        self.buffers.init(x0);

        // Save the true initial state before any processing
        if let Some(result) = result {
            result.insert(t, &self.x);
        }

        if let Some(manager) = writer_manager {
            // run the model function to update internal algebraic/kinematic states
            model.f(t, x0, &mut self.buffers.derivative)?;
            for event in &mut events.save_events {
                if event.options.every_step {
                    (event.save_fn)(model, &self.x, t, manager);
                }
            }
        }

        // Process initial events if any are scheduled at t0
        if events.process_periodic_events(model, &mut self.x, t) {
            // If initial events changed state, save the updated state
            if let Some(result) = result {
                result.insert(t, &self.x);
            }
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
            self.step(model, t, dt, false, &mut function_calls)?;

            // Update time based on dt
            t += dt;

            // Save the memory result state
            if let Some(result) = result {
                result.insert(t, &self.y);
            }
            if let Some(manager) = writer_manager {
                // run the model function to update internal algebraic/kinematic states
                model.f(t, x0, &mut self.buffers.derivative)?;
                for event in &mut events.save_events {
                    if event.options.every_step {
                        (event.save_fn)(model, &self.x, t, manager);
                    }
                }
            }

            // Run any events
            if events.process_periodic_events(model, &mut self.y, t) {
                // Save the result after events
                if let Some(result) = result {
                    result.insert(t, &self.y);
                }
            };

            // Initialize next loop
            self.x.clone_from(&self.y);

            if self.tableau.fsal {
                // Reuse last stage from previous step as first stage
                let (k0, ks) = self.buffers.stage.k.split_at_mut(1);
                k0[0].clone_from(ks.last().unwrap());
            }
        }

        // write the last state
        if let Some(manager) = writer_manager {
            for event in &mut events.save_events {
                if event.options.every_step {
                    (event.save_fn)(model, &self.x, t, manager);
                }
            }
        }
        Ok(())
    }

    /// Solves the system using adaptive step size control with event detection.
    ///
    /// Supports both periodic and continuous events, and performs root-finding
    /// to locate the time of continuous events using Brent's method.
    pub fn solve_adaptive<Model>(
        &mut self,
        model: &mut Model,
        x0: &State,
        tspan: (f64, f64),
        controller: &mut AdaptiveStepControl,
        events: &mut EventManager<Model, State>,
        result: &mut Option<MemoryResult<State>>,
        writer_manager: &mut Option<WriterManager>,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState + Adaptive,
    {
        let mut accept_counter = 0;
        let mut reject_counter = 0;
        let mut function_calls = 0;

        let mut t = tspan.0;

        // Copy initial state to all buffers to make sure length matches initial size of dynamically sized State
        self.x.clone_from(x0);
        self.y.clone_from(x0);
        self.y_tilde.clone_from(x0);
        self.buffers.init(x0);

        let mut dt = 1e-3; // initial dt

        // Save the true initial state before any processing
        if let Some(result) = result {
            result.insert(t, &self.x);
        }
        if let Some(manager) = writer_manager {
            // run the model function to update any mutable internal algebraic/kinematic states
            model.f(t, x0, &mut self.buffers.derivative)?;
            for event in &mut events.save_events {
                if event.options.every_step {
                    (event.save_fn)(model, &self.x, t, manager);
                }
            }
        }

        // // Process initial events if any are scheduled at t0
        // if events.process_periodic_events(model, &mut self.x, t) {
        //     // If initial events changed state, save the updated state
        //     result.save(t, &self.x)?;
        // };

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
            self.step(model, t, dt, true, &mut function_calls)?;

            // Calculate error
            let error = self.y.compute_error(
                &self.x,
                &self.y_tilde,
                controller.abs_tol,
                controller.rel_tol,
            );

            // Calculate new step size based on dt
            let new_dt = controller.step(dt, error, ORDER);

            // Check if step is accepted
            if error <= 1.0 {
                // Step ACCEPTED: advance time, save result, and update state

                // First determine if any continuous events occurred that require us to step back in time
                let mut continuous_event_occurred = false;
                let mut continuous_event_time = INFINITY;
                let mut event_indices = Vec::new();
                for (i, event) in events.continuous_events.iter_mut().enumerate() {
                    let (event_occurred, event_time) = self.continuous_event_occurred(t, dt, event);
                    if event_occurred {
                        continuous_event_occurred = true;
                        if event_time < continuous_event_time {
                            continuous_event_time = event_time;
                            event_indices.clear();
                            event_indices.push(i);
                        }
                    }
                }
                if continuous_event_occurred {
                    self.interpolate(t, dt, continuous_event_time);
                    // save the result prior to the event action
                    if let Some(result) = result {
                        result.insert(continuous_event_time, &self.buffers.interpolant);
                    }
                    // perform the event actions
                    for i in event_indices {
                        (events.continuous_events[i].action)(
                            model,
                            &mut self.buffers.interpolant,
                            continuous_event_time,
                        );
                        // save after each action
                        if let Some(result) = result {
                            result.insert(continuous_event_time, &self.buffers.interpolant);
                        }
                    }
                    // update dt for the continuous event time
                    dt = continuous_event_time - t;
                    // update state for the interpolated state after all events have occurred
                    self.y.clone_from(&self.buffers.interpolant);
                }

                t += dt;
                // Save the true state before any event processing
                if let Some(result) = result {
                    result.insert(t, &self.y);
                }
                if let Some(manager) = writer_manager {
                    // run the model function to update internal algebraic/kinematic states
                    // this unfortunately incurs one more function call
                    // TODO: could put this in the step method after the first stage call to avoid extra function call
                    model.f(t, &self.y, &mut self.buffers.derivative)?;
                    for event in &mut events.save_events {
                        if event.options.every_step {
                            (event.save_fn)(model, &self.y, t, manager);
                        }
                    }
                }

                // Process periodic events if any occurred
                if events.process_periodic_events(model, &mut self.y, t) {
                    // Events changed state, save the updated state
                    if let Some(result) = result {
                        result.insert(t, &self.y);
                    }
                    // if let Some(manager) = writer_manager {
                    //     // run the model function to update internal algebraic/kinematic states
                    //     model.f(t, &self.y, &mut self.buffers.derivative)?;
                    //     for event in &mut events.save_events {
                    //         if event.options.every_step {
                    //             (event.save_fn)(model, &self.y, t, manager);
                    //         }
                    //     }
                    // }
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
        // write the last state
        if let Some(manager) = writer_manager {
            for event in &mut events.save_events {
                if event.options.every_step {
                    (event.save_fn)(model, &self.x, t, manager);
                }
            }
        }

        println!(
            "Adaptive step size: accepted {} steps, rejected {} steps, function calls: {}",
            accept_counter, reject_counter, function_calls
        );
        Ok(())
    }
    /// Performs a single integration step using the configured Butcher tableau.
    ///
    /// Supports FSAL and dense output (when `adaptive` is true).
    pub fn step<Model>(
        &mut self,
        model: &mut Model,
        t: f64,
        h: f64,
        adaptive: bool,
        function_calls: &mut i32,
    ) -> Result<(), Box<dyn Error>>
    where
        Model: OdeModel<State = State>,
        State: OdeState,
    {
        let k = &mut self.buffers.stage.k;

        if self.tableau.fsal {
            // FSAL method implementation
            if self.first_step {
                model.f(t, &self.x, &mut k[0])?;
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

                model.f(
                    t + self.tableau.c[s] * h,
                    &self.buffers.state,
                    &mut k[s],
                )?;
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
            model.f(t + h, &self.y, &mut k[STAGES - 1])?;
            *function_calls += 1;
        } else {
            // Standard (non-FSAL) method implementation
            model.f(t, &self.x, &mut k[0])?;
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

                model.f(
                    t + self.tableau.c[s] * h,
                    &self.buffers.state,
                    &mut k[s],
                )?;
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
        Ok(())
    }

    /// Computes the interpolated state at time `t` using dense output coefficients.
    ///
    /// # Panics
    ///
    /// Panics if interpolation coefficients are not available or if `t` is outside `[t0, t0+dt]`.
    pub fn interpolate(&mut self, t0: f64, dt: f64, t: f64) {
        if let Some(bi) = &self.tableau.bi {
            if t < t0 || t > t0 + dt {
                panic!("t out of range for interpolation - todo extrapolation?");
            }

            // Reset interpolant buffer
            self.buffers.interpolant *= 0.0;

            // Calculate theta
            let theta = (t - t0) / dt;

            for s in 0..STAGES {
                // Evaluate polynomial using Horner's method
                let mut b = bi[s][ORDER - 2]; // highest degree coefficient
                for i in (0..ORDER - 2).rev() {
                    b = b * theta + bi[s][i];
                }

                // Apply theta multiplier
                b *= theta;

                // Scale and accumulate interpolant
                self.buffers
                    .derivative
                    .clone_from(&self.buffers.stage.k[s]);
                self.buffers.derivative *= b;
                self.buffers.interpolant += &self.buffers.derivative;
            }

            // Final interpolated value: x + dt * sum(b_s * k_s)
            self.buffers.interpolant *= dt;
            self.buffers.interpolant += &self.x;
        } else {
            panic!("No interpolation coefficients for solver");
        }
    }

    /// Determines whether a continuous event condition crosses zero during the last step.
    ///
    /// Uses Brent's method for robust root finding. Returns a tuple:
    /// - `bool`: Whether the event occurred.
    /// - `f64`: The estimated time of the zero crossing.
    fn continuous_event_occurred<Model: OdeModel<State = State>>(
        &mut self,
        t0: f64, // time at beginning of step
        dt: f64, // time at end of step
        event: &mut ContinuousEvent<Model, State>,
    ) -> (bool, f64) {
        if event.first_pass {
            // need to calculate original value
            event.last_check = (event.condition)(&self.x, t0);
            event.first_pass = false;
        }

        let mut fb = (event.condition)(&self.y, t0 + dt);
        if event.last_check * fb > 0.0 {
            // event did not occur
            event.last_check = fb;
            return (false, t0 + dt);
        }
        // other wise event occurred, use brent's method to determine exactly when
        let mut a = t0;
        let mut b = t0 + dt;

        let mut fa = event.last_check;
        if fa.abs() < fb.abs() {
            swap(&mut a, &mut b);
            swap(&mut fa, &mut fb);
        }

        let mut c = a;
        let mut mflag = true;
        let mut fs = INFINITY;
        let mut d = a;
        let mut output = b;
        let max_iters = 50;
        let mut iter = 0;

        while fb.abs() > event.tol && fs.abs() > event.tol {
            iter += 1;
            if iter > max_iters {
                panic!("max iters reached on brents method for continuous event interpolation")
            }
            self.interpolate(t0, dt, c);
            let fc = (event.condition)(&self.buffers.interpolant, c);
            let mut s = if fa != fc && fb != fc {
                // inverse quadratic interpolation
                a * fb * fc / ((fa - fb) * (fa - fc))
                    + b * fa * fc / ((fb - fa) * (fb - fc))
                    + c * fa * fb / ((fc - fa) * (fc - fb))
            } else {
                //secant method
                b - fb * (b - a) / (fb - fa)
            };

            if (s < (3.0 * a + b) / 4.0 && s > b)
                || (s < b && s > (3.0 * a + b) / 4.0)
                || (mflag && (s - b).abs() >= (b - c).abs() / 2.0)
                || (!mflag && (s - b).abs() >= (c - d) / 2.0)
                || (mflag && (b - c).abs() < event.tol)
                || (!mflag && (c - d).abs() < event.tol)
            {
                // bisection method
                s = (a + b) / 2.0;
                mflag = true;
            } else {
                mflag = false;
            }
            self.interpolate(t0, dt, s);

            fs = (event.condition)(&self.buffers.interpolant, s);
            d = c;
            c = b;
            if fa * fs < 0.0 {
                b = s;
                // already interped with s above
                fb = (event.condition)(&self.buffers.interpolant, b);
            } else {
                a = s;
                // already interped with s above
                fa = (event.condition)(&self.buffers.interpolant, a);
            }

            if fa.abs() < fb.abs() {
                swap(&mut a, &mut b);
                swap(&mut fa, &mut fb);
            }

            if fb.abs() < event.tol {
                output = b;
                break;
            }

            if fs.abs() < event.tol {
                output = s;
                break;
            }
        }
        (true, output)
    }
}
/// A buffer holding the `k` stages (derivative evaluations) for a Runge-Kutta method.
#[derive(Debug, Clone)]
pub struct StageBuffer<State: OdeState, const STAGES: usize> {
    pub k: [State; STAGES],
}

impl<State: OdeState, const STAGES: usize> StageBuffer<State, STAGES> {
    fn new() -> Self {
        Self { k: array::from_fn(|_| State::default()) }
    }
}
