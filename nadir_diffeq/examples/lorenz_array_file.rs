use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    saving::SaveMethod,
    state::state_array::StateArray,
    stepping::{AdaptiveStepControl, StepMethod},
};
use std::{env::current_dir, error::Error};

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
    // Create the model
    let model = Lorenz {
        sigma: 10.,
        rho: 28.,
        beta: 8. / 3.,
    };

    // Create the ode problem
    let mut problem = OdeProblem::new(
        model,
        Solver::Tsit5,
        StepMethod::Adaptive(AdaptiveStepControl::default()),
        SaveMethod::File {
            root_folder: current_dir()?.join("lorenz_example_results"),
        },
    );

    // Solve the problem with some initial condition and tspan
    let x0 = StateArray::new([1.0, 0.0, 0.0]); // Initial conditions for x, y, z
    problem.solve(&x0, (0.0, 10.0))?;

    // REMEMBER TO MANAGE YOUR RESULT FILES AND NOT COMMIT THEM TO YOUR REPO ON ACCIDENT!!!
    Ok(())
}
