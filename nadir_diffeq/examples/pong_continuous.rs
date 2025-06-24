use std::error::Error;

use nadir_diffeq::{
    OdeModel, OdeProblem,
    events::ContinuousEvent,
    saving::ResultStorage,
    solvers::{OdeSolver, RungeKuttaMethods},
    state::state_array::StateArray,
    stepping::AdaptiveStepControl,
};

#[derive(Debug)]
struct Pong {
    speed: f64,
}

impl OdeModel for Pong {
    type State = StateArray<1>;

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

    let problem = OdeProblem::new(model).with_continuous_event(ContinuousEvent::new(
        |x: &StateArray<1>, _t| x[0].abs() - 1.0,
        |model: &mut Pong, _state, _t| {
            model.speed *= -1.0;
        },
    ));
    let solver = OdeSolver::new(RungeKuttaMethods::Tsit5.into());

    let result = solver.solve_adaptive(
        problem,
        x0,
        (0.0, 10.0),
        AdaptiveStepControl::default(),
    )?;

    match result {
        ResultStorage::Memory(result) => {
            for i in 0..result.t.len() {
                println!(
                    "{:10.6}     {:10.6} ",
                    result.t[i], result.y[i][0]
                );
            }
        }
        _ => {}
    }
    Ok(())
}
