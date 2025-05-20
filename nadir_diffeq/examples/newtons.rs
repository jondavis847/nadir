use std::time::Instant;

use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    saving::{ResultStorage, SaveMethod},
    state_array::StateArray,
    stepping::{AdaptiveStepControl, StepMethod},
};

#[derive(Debug)]
struct Newtons {
    mu: f64,
}

impl OdeModel<StateArray<6>> for Newtons {
    fn f(&mut self, _t: f64, y: &StateArray<6>, dy: &mut StateArray<6>) {
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
    }
}

fn main() {
    let model = Newtons { mu: 3.986004415e14 };
    // Initial conditions polar spacecraft
    // let x0 = StateArray::new([
    //     -1821886.532,
    //     -5428723.719,
    //     4114923.555,
    //     -2636.498864,
    //     -3681.587186,
    //     -6004.153232,
    // ]);

    // Initial conditions hiehgly elliptic
    let x0 = StateArray::new([350000.0, 0.0, 0.0, 0.0, 47125.08767479528, 0.0]);

    let mut solver = OdeProblem::new(
        model,
        Solver::Verner6,
        StepMethod::Adaptive(
            AdaptiveStepControl::default()
                .with_rel_tol(1e-14)
                .with_abs_tol(1e-14),
        ),
        //StepMethod::Fixed(FixedStepControl::new(0.1)),
        SaveMethod::Memory,
    );

    let start = Instant::now();
    let result = solver.solve(&x0, (0.0, 86400.0 * 25.0));
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
