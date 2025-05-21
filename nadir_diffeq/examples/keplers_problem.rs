use std::{path::PathBuf, time::Instant};

use aerospace::orbit::KeplerianElements;
use celestial::CelestialBodies;
use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    saving::{ResultStorage, SaveMethod},
    state_array::StateArray,
    stepping::{AdaptiveStepControl, StepMethod},
};
use time::{Time, TimeSystem};

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
    // Initial conditions highly elliptic
    let orbit = KeplerianElements::new(
        7e6,
        0.96,
        0.0,
        0.0,
        0.0,
        0.0,
        Time::from_sec_j2k(0.0, TimeSystem::UTC),
        CelestialBodies::Earth,
    );
    let (r, v) = orbit.get_rv();
    let x0 = StateArray::new([r[0], r[1], r[2], v[0], v[1], v[2]]);

    let result_path = std::env::current_dir().unwrap().join("results");

    let mut solver = OdeProblem::new(
        model,
        Solver::Tsit5,
        StepMethod::Adaptive(
            AdaptiveStepControl::default()
                .with_rel_tol(1e-14)
                .with_abs_tol(1e-14),
        ),
        //StepMethod::Fixed(FixedStepControl::new(0.1)),
        //        SaveMethod::Memory,
        SaveMethod::File(result_path),
    );

    let new_orbit = orbit
        .keplers_problem(Time::from_sec_j2k(1472092.8448219472, TimeSystem::UTC))
        .unwrap();
    let (r, v) = new_orbit.get_rv();
    dbg!(r);
    dbg!(v);

    let start = Instant::now();
    let result = solver.solve(&x0, (0.0, 1472092.8448219472));
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
