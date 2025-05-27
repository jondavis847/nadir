use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    saving::{SaveMethod, WriterId, WriterManager},
    state::{Integrable, state_array::StateArray},
    stepping::{FixedStepControl, StepMethod},
};
use std::error::Error;

#[derive(Debug)]
struct Lorenz {
    sigma: f64,
    rho: f64,
    beta: f64,
    writers: [WriterId; 3],
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

impl WritableModel for Lorenz {
    fn init_writers(&mut self, manager: &mut WriterManager) -> Result<(), Box<dyn Error>> {
        for i in 0..3 {
            let file_name = format!("x{i}.csv");
            let id = manager.add_writer(manager.dir_path().join(file_name))?;
            self.writers[i] = id;
        }
        Ok(())
    }

    fn write_record(&self, t: f64, x: &StateArray<3>, manager: &mut WriterManager) {
        for i in 0..3 {
            if let Some(buffer) = manager.get_buffer(&self.writers[i]) {
                write!(buffer[0], x[i]);
            }
        }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let model = Lorenz {
        sigma: 10.,
        rho: 28.,
        beta: 8. / 3.,
        writers: [WriterId::default(); 3],
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
