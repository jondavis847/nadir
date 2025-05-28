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
    writers: [WriterId; 3],
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

    fn init_writers(&mut self, manager: &mut WriterManagerBuilder) -> Result<(), Box<dyn Error>> {
        for i in 0..3 {
            let file_name = format!("x{i}.csv");
            let id = manager.add_writer(manager.dir_path.join(file_name), 2)?;
            self.writers[i] = id;
        }
        Ok(())
    }

    fn write_record(
        &self,
        t: f64,
        x: &StateArray<3>,
        manager: &mut WriterManager,
    ) -> Result<(), Box<dyn Error>> {
        for i in 0..3 {
            let writer = manager.get_writer(&self.writers[i]);
            writer.write_column(0, t)?;
            writer.write_column(0, x[i])?;
        }

        Ok(())
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut model = Lorenz {
        sigma: 10.,
        rho: 28.,
        beta: 8. / 3.,
        writers: [WriterId::default(); 3],
    };

    let path = std::env::current_dir()?.join("lorenz_results");
    let mut writer_manager = WriterManagerBuilder::new(path);
    model.init_writers(&mut writer_manager)?;

    let mut problem = OdeProblem::new(
        model,
        Solver::Tsit5,
        StepMethod::Adaptive(AdaptiveStepControl::default()),
        SaveMethod::File(writer_manager),
    );

    let x0 = StateArray::new([1.0, 0.0, 0.0]); // Initial conditions for x, y, z{

    problem.solve(&x0, (0.0, 10.0))?;

    Ok(())
}
