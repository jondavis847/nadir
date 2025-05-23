use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    saving::SaveMethod,
    state::{StateWriterBuilder, state_array::StateArray},
    stepping::{FixedStepControl, StepMethod},
};
use std::error::Error;
use std::fmt::Write;

#[derive(Debug)]
struct Lorenz {
    sigma: f64,
    rho: f64,
    beta: f64,
}

impl OdeModel<StateArray<3>> for Lorenz {
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

    let path = std::env::current_dir().unwrap().join("results.csv");

    let mut problem = OdeProblem::new(
        model,
        Solver::Tsit5,
        StepMethod::Fixed(FixedStepControl::new(0.1)),
        SaveMethod::File(StateArray::<3>::writer(path).with_headers(["t", "x", "y", "z"])?),
    );

    let x0 = StateArray::new([1.0, 0.0, 0.0]); // Initial conditions for x, y, z{

    problem.solve(&x0, (0.0, 10.0))?;

    Ok(())
}
