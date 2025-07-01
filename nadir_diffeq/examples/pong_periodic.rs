use std::error::Error;

use nadir_diffeq::{
    OdeProblem,
    events::PeriodicEvent,
    model::OdeModel,
    solvers::{OdeSolver, RungeKuttaMethods},
    state::state_array::StateArray,
    stepping::AdaptiveStepControl,
};
use plotting::{QuickPlot, figure::Figure, line::Line};

#[derive(Debug, Clone)]
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

    let problem = OdeProblem::new(model).with_periodic_event(PeriodicEvent::new(
        1.0,
        0.0,
        |model: &mut Pong, _state, _t| {
            model.speed *= -1.0;
        },
    ));
    let solver = OdeSolver::new(RungeKuttaMethods::Tsit5.into());
    let result = solver
        .solve_adaptive(
            problem,
            x0,
            (0.0, 10.0),
            AdaptiveStepControl::default()
                .with_abs_tol(1e-6)
                .with_rel_tol(1e-6),
        )?
        .unwrap();

    let mut f = Figure::new();
    let a = f.get_axes(0)?;
    let y: Vec<f64> = result
        .y
        .iter()
        .map(|x| x[0])
        .collect();

    a.add_line(Line::new(&result.t, &y)?);

    QuickPlot::plot(f)?;

    Ok(())
}
