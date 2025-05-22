use std::time::Instant;

use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    events::ContinuousEvent,
    saving::{ResultStorage, SaveMethod},
    state_array::StateArray,
    stepping::{AdaptiveStepControl, StepMethod},
};

#[derive(Debug)]
struct Pong {
    speed: f64,
}

impl OdeModel<StateArray<1>> for Pong {
    fn f(&mut self, _t: f64, _y: &StateArray<1>, dy: &mut StateArray<1>) {
        dy[0] = self.speed;
    }
}

fn main() {
    let model = Pong { speed: 1.0 };

    // Initial conditions for elliptical orbit
    let x0 = StateArray::new([0.0]);

    let mut problem = OdeProblem::new(
        model,
        Solver::Verner9,
        StepMethod::Adaptive(
            AdaptiveStepControl::default()
                .with_rel_tol(1e-6)
                .with_abs_tol(1e-9),
        ),
        SaveMethod::Memory,
    )
    .with_event_continuous(ContinuousEvent::new(
        |x: &StateArray<1>, _t| x[0].abs() - 1.0,
        |model: &mut Pong, _state, _t| {
            model.speed *= -1.0;
        },
    ));
    let start = Instant::now();
    let result = problem.solve(&x0, (0.0, 10.0));
    let stop = Instant::now();
    dbg!(stop.duration_since(start).as_secs_f64());

    match result {
        ResultStorage::Memory(result) => {
            for i in 0..result.t.len() {
                println!("{:10.6}     {:10.6} ", result.t[i], result.y[i][0]);
            }
        }
        _ => {}
    }
}
