use std::{error::Error, f64::INFINITY};

use crate::{OdeModel, saving::WriterManager, state::OdeState};

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
    pub periodic_events: Vec<PeriodicEvent<Model, State>>,
    /// List of save events occurring at some frequency.
    pub save_events: Vec<SaveEvent<Model, State>>,
    /// List of events to run at the beginning of the sim.
    pub presim_events: Vec<PreSimEvent<Model, State>>,
    /// List of events to run at the end of the sim.
    pub postsim_events: Vec<PostSimEvent<Model>>,
    /// Next scheduled periodic event time and its indices.
    next_periodic: NextEvent,
    /// Placeholder for future discrete events (not yet implemented).
    next_discrete: NextEvent,
}

// Manual implementation so that Model and State  are not required to be Clone
impl<Model, State> Clone for EventManager<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    fn clone(&self) -> Self {
        Self {
            continuous_events: self
                .continuous_events
                .clone(),
            periodic_events: self
                .periodic_events
                .clone(),
            save_events: self
                .save_events
                .clone(),
            presim_events: self
                .presim_events
                .clone(),
            postsim_events: self
                .postsim_events
                .clone(),
            next_periodic: self
                .next_periodic
                .clone(),
            next_discrete: self
                .next_discrete
                .clone(),
        }
    }
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
            save_events: Vec::new(),
            presim_events: Vec::new(),
            postsim_events: Vec::new(),
            next_periodic: NextEvent { next_time: INFINITY, index: Vec::new() },
            next_discrete: NextEvent { next_time: INFINITY, index: Vec::new() },
        }
    }

    /// Add a new continuous event that is evaluated every step.
    pub fn add_continuous(&mut self, event: ContinuousEvent<Model, State>) {
        self.continuous_events
            .push(event);
    }

    /// Add a periodic event and update the internal schedule.
    pub fn add_periodic(&mut self, event: PeriodicEvent<Model, State>) {
        self.periodic_events
            .push(event);
        self.find_next_periodic();
    }

    /// Add a new continuous event that is evaluated every step.
    pub fn add_save(&mut self, event: SaveEvent<Model, State>) {
        self.save_events
            .push(event);
    }

    /// Add a new presim event that is evaluated once before the start of the simulation.
    pub fn add_presim(&mut self, event: PreSimEvent<Model, State>) {
        self.presim_events
            .push(event);
    }

    /// Add a new postsim event that is evaluated once at the end of the simulation.
    pub fn add_postsim(&mut self, event: PostSimEvent<Model>) {
        self.postsim_events
            .push(event);
    }

    /// Returns the time of the next scheduled event (periodic or discrete).
    pub fn next_time(&self) -> f64 {
        self.next_periodic
            .next_time
            .min(
                self.next_discrete
                    .next_time,
            )
    }

    /// Updates internal record of which periodic events are next.
    fn find_next_periodic(&mut self) {
        if self
            .periodic_events
            .is_empty()
        {
            return;
        }

        self.next_periodic
            .next_time = INFINITY;
        self.next_periodic
            .index
            .clear();

        let mut min_time = self.periodic_events[0].next_time;
        self.next_periodic
            .index
            .push(0);

        for (i, event) in self
            .periodic_events
            .iter()
            .enumerate()
            .skip(1)
        {
            let time_diff = event.next_time - min_time;

            if time_diff.abs() < 1e-12 {
                self.next_periodic
                    .index
                    .push(i);
            } else if time_diff < 0.0 {
                min_time = event.next_time;
                self.next_periodic
                    .index
                    .clear();
                self.next_periodic
                    .index
                    .push(i);
            }
        }

        self.next_periodic
            .next_time = min_time;
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

        if !self
            .periodic_events
            .is_empty()
            && t >= self
                .next_periodic
                .next_time
        {
            periodic_event_occurred = true;

            for i in &self
                .next_periodic
                .index
            {
                let event = &mut self.periodic_events[*i];
                event.perform_event(model, state, t);
            }

            self.find_next_periodic();
        }

        periodic_event_occurred
    }
}

/// Internal struct for tracking the next event time and indices of tied events.
#[derive(Clone)]
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
    f: fn(&mut Model, &mut State, f64),
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
    pub fn new(period: f64, start_time: f64, f: fn(&mut Model, &mut State, f64)) -> Self {
        Self { period, next_time: start_time, f }
    }

    /// Triggers the event and schedules the next one based on its period.
    pub fn perform_event(&mut self, model: &mut Model, state: &mut State, t: f64) {
        (self.f)(model, state, t);
        self.next_time = t + self.period;
    }
}

impl<Model, State> Clone for PeriodicEvent<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    fn clone(&self) -> Self {
        Self {
            period: self.period,
            next_time: self.next_time,
            f: self.f, // Function pointers implement Copy/Clone
        }
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
    pub condition: fn(&State, f64) -> f64,
    /// Action to perform when the zero-crossing is detected.
    pub action: fn(&mut Model, &mut State, f64),
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
    pub fn new(condition: fn(&State, f64) -> f64, action: fn(&mut Model, &mut State, f64)) -> Self {
        Self {
            last_check: 1.0,
            first_pass: true,
            condition,
            action,
            tol: 1e-6,
        }
    }

    /// Sets the tolerance used to detect condition crossings.
    pub fn with_tol(mut self, tol: f64) -> Self {
        self.tol = tol;
        self
    }
}

impl<Model, State> Clone for ContinuousEvent<Model, State>
where
    Model: OdeModel<State = State>,
    State: OdeState,
{
    fn clone(&self) -> Self {
        Self {
            last_check: self.last_check,
            first_pass: self.first_pass,
            condition: self.condition, // Function pointer implements Copy/Clone
            action: self.action,       // Function pointer implements Copy/Clone
            tol: self.tol,
        }
    }
}

#[derive(Clone)]
pub struct SaveEventOptions {
    pub every_step: bool,
    pub every_event: bool,
    pub periodic: Option<f64>,
}

impl Default for SaveEventOptions {
    fn default() -> Self {
        Self { every_step: true, every_event: true, periodic: None }
    }
}

/// SaveEvents will interpolate in between adaptive steps, even when set to periodic
pub struct SaveEvent<Model, State> {
    pub options: SaveEventOptions,
    pub init_fn: fn(&mut Model, &State, &mut WriterManager),
    pub save_fn: fn(&Model, &State, f64, &mut WriterManager),
}

impl<Model, State> SaveEvent<Model, State> {
    pub fn new(
        init_fn: fn(&mut Model, &State, &mut WriterManager),
        save_fn: fn(&Model, &State, f64, &mut WriterManager),
    ) -> Self {
        Self { options: SaveEventOptions::default(), init_fn, save_fn }
    }

    pub fn with_options(mut self, options: SaveEventOptions) -> Self {
        self.options = options;
        self
    }
}

impl<Model, State> Clone for SaveEvent<Model, State> {
    fn clone(&self) -> Self {
        Self {
            options: self
                .options
                .clone(),
            init_fn: self.init_fn, // Function pointer implements Copy/Clone
            save_fn: self.save_fn, // Function pointer implements Copy/Clone
        }
    }
}

pub struct PreSimEvent<Model, State> {
    pub f: fn(&mut Model, &State, f64, &Option<WriterManager>) -> Result<(), Box<dyn Error>>,
}

impl<Model, State> PreSimEvent<Model, State> {
    pub fn new(
        f: fn(&mut Model, &State, f64, &Option<WriterManager>) -> Result<(), Box<dyn Error>>,
    ) -> Self {
        Self { f }
    }
}

impl<Model, State> Clone for PreSimEvent<Model, State> {
    fn clone(&self) -> Self {
        Self {
            f: self.f, // Function pointer implements Copy/Clone
        }
    }
}

pub struct PostSimEvent<Model> {
    pub f: fn(&Model, &Option<WriterManager>),
}

impl<Model> PostSimEvent<Model> {
    pub fn new(f: fn(&Model, &Option<WriterManager>)) -> Self {
        Self { f }
    }
}

impl<Model> Clone for PostSimEvent<Model> {
    fn clone(&self) -> Self {
        Self {
            f: self.f, // Function pointer implements Copy/Clone
        }
    }
}
