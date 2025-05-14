use crate::{Integrable, OdeModel};

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
    f: Box<dyn Fn(&mut Model, &mut State, &mut f64, &mut f64)>, //(model,state,t)
}

impl<Model, State> PeriodicEvent<Model, State>
where
    Model: OdeModel<State>,
    State: Integrable,
{
    pub fn new<F>(period: f64, start_time: f64, f: F) -> Self
    where
        F: Fn(&mut Model, &mut State, &mut f64, &mut f64) + 'static,
    {
        Self {
            period,
            next_time: start_time,
            f: Box::new(f),
        }
    }

    pub fn has_occurred(&self, t: f64) -> bool {
        t >= self.next_time
    }

    pub fn perform_event(
        &mut self,
        model: &mut Model,
        state: &mut State,
        t: &mut f64,
        dt: &mut f64,
    ) {
        (self.f)(model, state, t, dt);
        self.next_time += self.period;
    }
}
