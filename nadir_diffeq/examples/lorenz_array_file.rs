use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    saving::{SaveMethod, WriterId, WriterManager, WriterManagerBuilder},
    state::state_array::StateArray,
    stepping::{AdaptiveStepControl, StepMethod},
};
use std::error::Error;

#[derive(Debug)]
struct Lorenz {
    sigma: f64,
    rho: f64,
    beta: f64,
    writer_id: WriterId,
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

    fn write_record(
        &self,
        t: f64,
        x: &StateArray<3>,
        manager: &mut WriterManager,
    ) -> Result<(), Box<dyn Error>> {
        let writer = manager.get_writer(&self.writer_id);
        writer.write_column(0, t)?;
        for i in 0..3 {
            writer.write_column(i + 1, x[i])?;
        }
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    // Create the file writers for the model
    let mut writer_manager =
        WriterManagerBuilder::new(std::env::current_dir()?.join("lorenz_results"));
    let writer_id = writer_manager
        .add_writer::<StateArray<3>>(writer_manager.dir_path.join("results.csv"), 4)?;

    // Create the model
    let model = Lorenz {
        sigma: 10.,
        rho: 28.,
        beta: 8. / 3.,
        writer_id,
    };

    // Create the ode problem
    let mut problem = OdeProblem::new(
        model,
        Solver::Tsit5,
        StepMethod::Adaptive(AdaptiveStepControl::default()),
        SaveMethod::File(writer_manager),
    );

    // Solve the problem with some initial condition and tspan
    let x0 = StateArray::new([1.0, 0.0, 0.0]); // Initial conditions for x, y, z
    problem.solve(&x0, (0.0, 10.0))?;

    // REMEMBER TO MANAGE YOUR RESULT FILES AND NOT COMMIT THEM TO YOUR REPO ON ACCIDENT!!!
    Ok(())
}
