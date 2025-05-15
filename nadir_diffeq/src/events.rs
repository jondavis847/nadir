use std::f64::INFINITY;

use crate::{Integrable, OdeModel};

pub struct EventManager<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable,
{
    periodic_events: Vec<PeriodicEvent<Model, State>>,
    next_periodic: NextEvent,
    next_discrete: NextEvent,
}

impl<Model, State> EventManager<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable,
{
    pub fn new() -> Self {
        Self {
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
        for (i, event) in self.periodic_events.iter().enumerate() {
            // reset
            self.next_periodic.next_time = INFINITY;

            // search for event with lowest time
            if event.next_time < self.next_periodic.next_time {
                self.next_periodic.index = vec![i];
                self.next_periodic.next_time = event.next_time;
            }

            // add event if it's very close to the next time
            if (event.next_time - self.next_periodic.next_time).abs() < 1e-12 {
                self.next_periodic.index.push(i);
                self.next_periodic.next_time = event.next_time;
            }
        }
    }

    pub fn process_events(&mut self, model: &mut Model, state: &mut State, t: &mut f64) -> bool {
        let mut event_occurred = false;
        // process periodic events
        if *t >= self.next_periodic.next_time {
            event_occurred = true;
            for i in &self.next_periodic.index {
                let event = &mut self.periodic_events[*i];
                event.perform_event(model, state, t);
            }
        }
        if event_occurred {
            self.find_next_periodic();
        }
        event_occurred
    }
}

struct NextEvent {
    next_time: f64,
    index: Vec<usize>,
}

pub enum Events<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable,
{
    Periodic(PeriodicEvent<Model, State>),
}

pub struct PeriodicEvent<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable,
{
    pub period: f64,
    pub next_time: f64,
    f: Box<dyn Fn(&mut Model, &mut State, &mut f64)>, //(model,state,t)
}

impl<Model, State> PeriodicEvent<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable,
{
    pub fn new<F>(period: f64, start_time: f64, f: F) -> Self
    where
        F: Fn(&mut Model, &mut State, &mut f64) + 'static,
    {
        Self {
            period,
            next_time: start_time,
            f: Box::new(f),
        }
    }

    pub fn perform_event(&mut self, model: &mut Model, state: &mut State, t: &mut f64) {
        (self.f)(model, state, t);
        self.next_time = *t + self.period;
    }
}
