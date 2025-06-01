use std::f64::INFINITY;

use crate::{OdeModel, state::OdeState};

/// Manages time-based events during ODE integration.
///
/// Supports both:
/// - **Periodic events** that occur at fixed intervals.
/// - **Continuous events** that trigger when a condition crosses zero.
pub struct EventManager<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    /// List of continuous events checked every step.
    pub continuous_events: Vec<ContinuousEvent<Model, State>>,
    /// List of periodic events occurring at fixed time intervals.
    periodic_events: Vec<PeriodicEvent<Model, State>>,
    /// Next scheduled periodic event time and its indices.
    next_periodic: NextEvent,
    /// Placeholder for future discrete events (not yet implemented).
    next_discrete: NextEvent,
}

impl<Model, State> EventManager<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    /// Constructs a new `EventManager` with no registered events.
    pub fn new() -> Self {
        Self {
            continuous_events: Vec::new(),
            periodic_events: Vec::new(),
            next_periodic: NextEvent {
                next_time: INFINITY,
                index: Vec::new(),
            },
            next_discrete: NextEvent {
                next_time: INFINITY,
                index: Vec::new(),
            },
        }
    }

    /// Add a new continuous event that is evaluated every step.
    pub fn add_continuous(&mut self, event: ContinuousEvent<Model, State>) {
        self.continuous_events.push(event);
    }

    /// Add a periodic event and update the internal schedule.
    pub fn add_periodic(&mut self, event: PeriodicEvent<Model, State>) {
        self.periodic_events.push(event);
        self.find_next_periodic();
    }

    /// Returns the time of the next scheduled event (periodic or discrete).
    pub fn next_time(&self) -> f64 {
        self.next_periodic
            .next_time
            .min(self.next_discrete.next_time)
    }

    /// Updates internal record of which periodic events are next.
    fn find_next_periodic(&mut self) {
        if self.periodic_events.is_empty() {
            return;
        }

        self.next_periodic.next_time = INFINITY;
        self.next_periodic.index.clear();

        let mut min_time = self.periodic_events[0].next_time;
        self.next_periodic.index.push(0);

        for (i, event) in self.periodic_events.iter().enumerate().skip(1) {
            let time_diff = event.next_time - min_time;

            if time_diff.abs() < 1e-12 {
                self.next_periodic.index.push(i);
            } else if time_diff < 0.0 {
                min_time = event.next_time;
                self.next_periodic.index.clear();
                self.next_periodic.index.push(i);
            }
        }

        self.next_periodic.next_time = min_time;
    }

    /// Executes any periodic events that are scheduled to occur at or before time `t`.
    ///
    /// Returns `true` if any event was triggered.
    pub fn process_periodic_events(
        &mut self,
        model: &mut Model,
        state: &mut State,
        t: f64,
    ) -> bool {
        let mut periodic_event_occurred = false;

        if !self.periodic_events.is_empty() && t >= self.next_periodic.next_time {
            periodic_event_occurred = true;

            for i in &self.next_periodic.index {
                let event = &mut self.periodic_events[*i];
                event.perform_event(model, state, t);
            }

            self.find_next_periodic();
        }

        periodic_event_occurred
    }
}

/// Internal struct for tracking the next event time and indices of tied events.
struct NextEvent {
    next_time: f64,
    index: Vec<usize>,
}

/// Represents a user-defined action to perform periodically at fixed time intervals.
pub struct PeriodicEvent<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    /// Period between event triggers.
    pub period: f64,
    /// Time at which the event is next scheduled to run.
    pub next_time: f64,
    /// The function to call when the event is triggered.
    f: Box<dyn FnMut(&mut Model, &mut State, f64)>,
}

impl<Model, State> PeriodicEvent<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    /// Creates a new `PeriodicEvent`.
    ///
    /// # Arguments
    /// * `period` - Time interval between executions.
    /// * `start_time` - Initial trigger time.
    /// * `f` - Function to call at each trigger.
    pub fn new<F>(period: f64, start_time: f64, f: F) -> Self
    where
        F: FnMut(&mut Model, &mut State, f64) + 'static,
    {
        Self {
            period,
            next_time: start_time,
            f: Box::new(f),
        }
    }

    /// Triggers the event and schedules the next one based on its period.
    pub fn perform_event(&mut self, model: &mut Model, state: &mut State, t: f64) {
        (self.f)(model, state, t);
        self.next_time = t + self.period;
    }
}

/// Represents an event that triggers when a user-defined condition crosses zero.
///
/// Used for root-finding in continuous simulation contexts.
pub struct ContinuousEvent<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    /// The last time the condition was evaluated.
    pub last_check: f64,
    /// Flag to indicate if this is the first check (for setup).
    pub first_pass: bool,
    /// A function representing the condition. Should return a signed value.
    pub condition: Box<dyn Fn(&State, f64) -> f64>,
    /// Action to perform when the zero-crossing is detected.
    pub action: Box<dyn FnMut(&mut Model, &mut State, f64)>,
    /// Tolerance used to detect zero-crossing.
    pub tol: f64,
}

impl<Model, State> ContinuousEvent<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    /// Creates a new continuous event.
    ///
    /// # Arguments
    /// * `condition` - A function returning a value whose zero-crossing triggers the event.
    /// * `action` - The action to take when the event is triggered.
    pub fn new<C, A>(condition: C, action: A) -> Self
    where
        C: Fn(&State, f64) -> f64 + 'static,
        A: FnMut(&mut Model, &mut State, f64) + 'static,
    {
        Self {
            last_check: 1.0,
            first_pass: true,
            condition: Box::new(condition),
            action: Box::new(action),
            tol: 1e-6,
        }
    }

    /// Sets the tolerance used to detect condition crossings.
    pub fn with_tol(mut self, tol: f64) -> Self {
        self.tol = tol;
        self
    }
}
