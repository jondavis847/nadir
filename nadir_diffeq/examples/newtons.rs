use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    saving::SaveMethod,
    state_array::StateArray,
    stepping::{StepMethod, StepPIDControl},
};

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
    let model = Newtons { mu: 3.986e14 };
    // Initial conditions for elliptical orbit
    let x0 = StateArray::new([
        -1821886.532,
        -5428723.719,
        4114923.555,
        -2636.498864,
        -3681.587186,
        -6004.153232,
    ]);

    // Create a results directory in the project folder
    let mut results_dir = std::env::current_dir().unwrap();
    results_dir.push("results");
    // Create the directory if it doesn't exist
    if !results_dir.exists() {
        std::fs::create_dir_all(&results_dir).expect("Failed to create results directory");
    }

    let mut solver = OdeProblem::new(
        model,
        Solver::DoPri45,
        StepMethod::Adaptive(StepPIDControl::default().with_tolerances(1e-6, 1e-9)),
        SaveMethod::File(results_dir),
    );

    solver.solve(&x0, (0.0, 7200.0));
}
