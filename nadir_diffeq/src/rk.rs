use tolerance::Tolerance;

use crate::{Integrable, OdeModel, tableau::ButcherTableau};

pub struct RungeKutta<State: Integrable, const STAGES: usize> {
    stage_buffer: StageBuffer<State, STAGES>, // preallocated buffer for stage results
    calc_buffer_state: State,                 // preallocated buffer for calculations
    calc_buffer_derivative: State::Derivative,
    tableau: ButcherTableau<STAGES>,
    tolerances: State::Tolerance,
}

impl<State: Integrable, const STAGES: usize> RungeKutta<State, STAGES> {
    pub fn new(tableau: ButcherTableau<STAGES>) -> Self
    where
        State: Integrable,
    {
        Self {
            stage_buffer: StageBuffer {
                k: std::array::from_fn(|_| State::Derivative::default()),
            },
            calc_buffer_state: State::default(),
            calc_buffer_derivative: State::Derivative::default(),
            tableau,
            tolerances: State::Tolerance::default(),
        }
    }

    pub fn step<Model: OdeModel<State>>(
        &mut self,
        model: &mut Model,
        x0: &State,
        xf: &mut State,
        t: f64,
        h: f64,
    ) {
        let k = &mut self.stage_buffer.k;

        // k0
        model.f(t, x0, &mut k[0]);

        // k1 - ks
        for s in 1..STAGES {
            // in place calculation of intermediate points
            self.calc_buffer_state *= 0.0;
            // sum previous ks with appropriate scaling from tableau
            for i in 0..s {
                self.calc_buffer_derivative.clone_from(&k[i]);
                self.calc_buffer_derivative *= self.tableau.a[s][i];
                self.calc_buffer_state += &self.calc_buffer_derivative;
            }
            self.calc_buffer_state *= h;
            self.calc_buffer_state += x0;

            model.f(
                t + self.tableau.c[s] * h,
                &self.calc_buffer_state,
                &mut k[s],
            );
        }
        xf.clone_from(x0);
        for s in 0..STAGES {
            self.calc_buffer_derivative.clone_from(&k[s]);
            self.calc_buffer_derivative *= self.tableau.b[s] * h;
            *xf += &self.calc_buffer_derivative;
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
