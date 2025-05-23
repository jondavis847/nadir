use std::error::Error;

use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    events::PeriodicEvent,
    saving::{ResultStorage, SaveMethod},
    state::state_array::StateArray,
    stepping::{AdaptiveStepControl, StepMethod},
};

#[derive(Debug)]
struct Pong {
    speed: f64,
}

impl OdeModel<StateArray<1>> for Pong {
    fn f(
        &mut self,
        _t: f64,
        _y: &StateArray<1>,
        dy: &mut StateArray<1>,
    ) -> Result<(), Box<dyn Error>> {
        dy[0] = self.speed;
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let model = Pong { speed: 1.0 };

    // Initial conditions for elliptical orbit
    let x0 = StateArray::new([0.0]);

    let mut problem = OdeProblem::new(
        model,
        Solver::DoPri45,
        StepMethod::Adaptive(
            AdaptiveStepControl::default()
                .with_rel_tol(1e-6)
                .with_abs_tol(1e-9),
        ),
        SaveMethod::Memory,
    )
    .with_event_periodic(PeriodicEvent::new(
        1.0,
        0.0,
        |model: &mut Pong, _state, _t| {
            model.speed *= -1.0;
        },
    ));

    let result = problem.solve(&x0, (0.0, 10.0))?;

    match result {
        ResultStorage::Memory(result) => {
            for i in 0..result.t.len() {
                println!("{:10.6}     {:10.6} ", result.t[i], result.y[i][0]);
            }
        }
        _ => {}
    }

    Ok(())
}
