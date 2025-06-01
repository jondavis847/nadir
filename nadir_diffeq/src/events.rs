use std::f64::INFINITY;

use crate::{OdeModel, state::OdeState};

pub struct EventManager<Model, State>
where
    Model: OdeModel<State>,
    State: OdeState,
{
    pub continuous_events: Vec<ContinuousEvent<Model, State>>,
    periodic_events: Vec<PeriodicEvent<Model, State>>,
    next_periodic: NextEvent,
    next_discrete: NextEvent,
}

impl<Model, State> EventManager<Model, State>
where
    Model: OdeModel<State>,
    State: OdeState,
{
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

    pub fn add_continuous(&mut self, event: ContinuousEvent<Model, State>) {
        self.continuous_events.push(event);
    }

    pub fn add_periodic(&mut self, event: PeriodicEvent<Model, State>) {
        self.periodic_events.push(event);
        self.find_next_periodic();
    }

    pub fn next_time(&self) -> f64 {
        self.next_periodic
            .next_time
            .min(self.next_discrete.next_time)
    }

    fn find_next_periodic(&mut self) {
        if self.periodic_events.is_empty() {
            return;
        }
        // Reset
        self.next_periodic.next_time = INFINITY;
        self.next_periodic.index.clear();

        // First event as initial minimum
        let mut min_time = self.periodic_events[0].next_time;
        self.next_periodic.index.push(0);

        // Process remaining events
        for (i, event) in self.periodic_events.iter().enumerate().skip(1) {
            let time_diff = event.next_time - min_time;

            if time_diff.abs() < 1e-12 {
                // Event happens at essentially the same time
                self.next_periodic.index.push(i);
            } else if time_diff < 0.0 {
                // Found a new minimum time
                min_time = event.next_time;
                self.next_periodic.index.clear();
                self.next_periodic.index.push(i);
            }
        }

        // Set the final minimum time
        self.next_periodic.next_time = min_time;
    }

    pub fn process_periodic_events(
        &mut self,
        model: &mut Model,
        state: &mut State,
        t: f64,
    ) -> bool {
        let mut periodic_event_occurred = false;

        // Process periodic events
        if !self.periodic_events.is_empty() && t >= self.next_periodic.next_time {
            periodic_event_occurred = true;

            // Process each scheduled event
            for i in &self.next_periodic.index {
                let event = &mut self.periodic_events[*i];
                event.perform_event(model, state, t);
            }

            // Find next events after processing
            self.find_next_periodic();
        }

        periodic_event_occurred
    }
}

struct NextEvent {
    next_time: f64,
    index: Vec<usize>,
}

pub struct PeriodicEvent<Model, State>
where
    Model: OdeModel<State>,
    State: OdeState,
{
    pub period: f64,
    pub next_time: f64,
    f: Box<dyn FnMut(&mut Model, &mut State, f64)>, //(model,state,t)
}

impl<Model, State> PeriodicEvent<Model, State>
where
    Model: OdeModel<State>,
    State: OdeState,
{
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

    pub fn perform_event(&mut self, model: &mut Model, state: &mut State, t: f64) {
        (self.f)(model, state, t);
        self.next_time = t + self.period;
    }
}
pub struct ContinuousEvent<Model, State>
where
    Model: OdeModel<State>,
    State: OdeState,
{
    pub last_check: f64,
    pub first_pass: bool,
    pub condition: Box<dyn Fn(&State, f64) -> f64>,
    pub action: Box<dyn FnMut(&mut Model, &mut State, f64)>,
    pub tol: f64,
}

impl<Model, State> ContinuousEvent<Model, State>
where
    Model: OdeModel<State>,
    State: OdeState,
{
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

    pub fn with_tol(mut self, tol: f64) -> Self {
        self.tol = tol;
        self
    }
}
