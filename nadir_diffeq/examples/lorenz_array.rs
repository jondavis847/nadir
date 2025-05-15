use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    events::PeriodicEvent,
    saving::{ResultStorage, SaveMethod},
    state_array::StateArray,
    stepping::{FixedStepControl, StepMethod},
};

struct Lorenz {
    sigma: f64,
    rho: f64,
    beta: f64,
}

impl OdeModel<StateArray<3>> for Lorenz {
    fn f(&mut self, _t: f64, x: &StateArray<3>, dx: &mut StateArray<3>) {
        dx[0] = self.sigma * (x[1] - x[0]);
        dx[1] = x[0] * (self.rho - x[2]) - x[1];
        dx[2] = x[0] * x[1] - self.beta * x[2];
    }
}

fn main() {
    let model = Lorenz {
        sigma: 10.,
        rho: 28.,
        beta: 8. / 3.,
    };

    let mut problem = OdeProblem::new(
        model,
        Solver::Tsit5,
        StepMethod::Fixed(FixedStepControl::new(0.1)),
        SaveMethod::Memory,
    )
    .with_event_periodic(PeriodicEvent::new(
        1.0,
        0.0,
        |_model, state: &mut StateArray<3>, _t| {
            state[1] = 3.0;
        },
    ));

    let x0 = StateArray::new([1.0, 0.0, 0.0]); // Initial conditions for x, y, z{

    let result = problem.solve(&x0, (0.0, 10.0));
    match result {
        ResultStorage::Memory(result) => {
            for i in 0..result.t.len() {
                //if result.t[i].rem_euclid(0.3) < 1e-4 {
                println!(
                    "{:10.12}     {:10.12}     {:10.12}     {:10.12}", // 10 chars wide, 6 decimal places
                    result.t[i], result.y[i][0], result.y[i][1], result.y[i][2]
                );
                //}
            }
        }
        _ => {}
    }
}
