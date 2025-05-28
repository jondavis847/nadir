use std::{error::Error, time::Instant};

use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    saving::{ResultStorage, SaveMethod},
    state::state_array::StateArray,
    stepping::{AdaptiveStepControl, StepMethod},
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

fn main() {
    let model = KeplerianOrbit { mu: 3.986004415e14 };

    let x0 = StateArray::new([7e6, 0.0, 0.0, 0.0, 7546.053287267836, 0.0]);

    let mut solver = OdeProblem::new(
        model,
        Solver::Verner9,
        StepMethod::Adaptive(
            AdaptiveStepControl::default()
                .with_rel_tol(1e-14)
                .with_abs_tol(1e-14),
        ),
        SaveMethod::Memory,
    );

    let start = Instant::now();
    let result = solver.solve(&x0, (0.0, 1472092.8448219472)).unwrap();
    let stop = Instant::now();
    dbg!(stop.duration_since(start).as_secs_f64());

    match result {
        ResultStorage::Memory(result) => {
            let n = result.t.len() - 1;
            println!(
                "{:10.6}     {:10.6} {:10.6} {:10.6}",
                result.t[n], result.y[n][0], result.y[n][1], result.y[n][2]
            );
        }

        _ => {}
    }
}
