use nadir_diffeq::{
    OdeModel, OdeProblem, saving::SaveMethod, solvers::Solver, state::state_array::StateArray,
    stepping::AdaptiveStepControl,
};
use std::error::Error;

#[derive(Debug)]
struct Lorenz {
    sigma: f64,
    rho: f64,
    beta: f64,
}

impl OdeModel for Lorenz {
    type State = StateArray<3>;

    fn f(
        &mut self,
        _t: f64,
        x: &StateArray<3>,
        dx: &mut StateArray<3>,
    ) -> Result<(), Box<dyn Error>> {
        dx[0] = self.sigma * (x[1] - x[0]);
        dx[1] = x[0] * (self.rho - x[2]) - x[1];
        dx[2] = x[0] * x[1] - self.beta * x[2];
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let model = Lorenz {
        sigma: 10.,
        rho: 28.,
        beta: 8. / 3.,
    };

    let mut problem = OdeProblem::new(model);

    let x0 = StateArray::new([1.0, 0.0, 0.0]); // Initial conditions for x, y, z{

    problem.solve_adaptive(
        &x0,
        (0.0, 10.0),
        AdaptiveStepControl::default(),
        Solver::Tsit5,
        SaveMethod::Memory,
    )?;

    Ok(())
}
