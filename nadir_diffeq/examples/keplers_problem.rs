use std::{env::current_dir, error::Error};

use nadir_diffeq::{
    OdeModel, OdeProblem,
    events::SaveEvent,
    saving::{SaveMethod, StateWriterBuilder, WriterManager},
    solvers::Solver,
    state::state_array::StateArray,
    stepping::AdaptiveStepControl,
};

#[derive(Debug)]
struct KeplerianOrbit {
    mu: f64,
}

impl OdeModel for KeplerianOrbit {
    type State = StateArray<6>;

    fn f(
        &mut self,
        _t: f64,
        y: &StateArray<6>,
        dy: &mut StateArray<6>,
    ) -> Result<(), Box<dyn Error>> {
        let r = [y[0], y[1], y[2]];
        let v = [y[3], y[4], y[5]];
        let rmag = (r[0] * r[0] + r[1] * r[1] + r[2] * r[2]).sqrt();
        let rmag_cubed = rmag * rmag * rmag;

        let a = [
            -self.mu / rmag_cubed * r[0],
            -self.mu / rmag_cubed * r[1],
            -self.mu / rmag_cubed * r[2],
        ];
        dy[0] = v[0];
        dy[1] = v[1];
        dy[2] = v[2];
        dy[3] = a[0];
        dy[4] = a[1];
        dy[5] = a[2];
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let model = KeplerianOrbit { mu: 3.986004415e14 };

    let x0 = StateArray::new([7e6, 0.0, 0.0, 0.0, 7546.053287267836, 0.0]);

    OdeProblem::new(model)
        .with_saving(current_dir()?.join("results"))
        .with_save_event(SaveEvent::new(init_fn, save_fn))
        .solve_adaptive(
            &x0,
            (0.0, 1472092.8448219472),
            AdaptiveStepControl::default()
                .with_abs_tol(1e-14)
                .with_rel_tol(1e-14),
            Solver::Verner9,
            SaveMethod::Memory,
        )?;

    Ok(())
}

fn init_fn(_model: &mut KeplerianOrbit, _state: &StateArray<6>, manager: &mut WriterManager) {
    manager.add_writer(
        StateWriterBuilder::new(7, "results.csv".into())
            .with_headers(&["t", "rx", "ry", "rz", "vx", "vy", "vz"])
            .unwrap(),
    );
}

fn save_fn(_model: &KeplerianOrbit, state: &StateArray<6>, t: f64, manager: &mut WriterManager) {
    if let Some((_, writer)) = manager.writers.iter_mut().next() {
        writer.float_buffer[0] = t;
        for i in 0..6 {
            writer.float_buffer[i + 1] = state[i];
        }
        writer.write_record().unwrap();
    }
}
