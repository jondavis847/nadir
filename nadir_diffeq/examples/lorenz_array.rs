use nadir_diffeq::{
    OdeModel, OdeProblem, SaveMethod, Solver, StepMethod, result::ResultStorage,
    state_array::StateArray,
};

struct Lorenz {
    sigma: f64,
    rho: f64,
    beta: f64,
}

impl OdeModel<StateArray<3>> for Lorenz {
    fn f(&mut self, _t: f64, x: &StateArray<3>, dx: &mut StateArray<3>) {
        dx[0] = self.sigma * (x[1] - x[0]);
        dx[1] = x[0] * (self.rho - x[2]) - x[1];
        dx[2] = x[0] * x[1] - self.beta * x[2];
    }
}

fn main() {
    let mut model = Lorenz {
        sigma: 10.,
        rho: 28.,
        beta: 8. / 3.,
    };

    // Create a results directory in the project folder
    let mut results_dir = std::env::current_dir().unwrap();
    results_dir.push("results");

    // Create the directory if it doesn't exist
    if !results_dir.exists() {
        std::fs::create_dir_all(&results_dir).expect("Failed to create results directory");
    }

    let mut solver = OdeProblem::new(
        Solver::Tsit5,
        StepMethod::Fixed(0.001),
        SaveMethod::File(results_dir),
    );

    let x0 = StateArray::new([1.0, 0.0, 0.0]); // Initial conditions for x, y, z{

    let result = solver.solve(&mut model, &x0, (0.0, 30.0));
    match result {
        ResultStorage::Memory(result) => {
            for i in 0..result.t.len() {
                if result.t[i] - result.t[i].floor() < 1e-4 {
                    println!(
                        "{:10.6}     {:10.6}     {:10.6}     {:10.6}", // 10 chars wide, 6 decimal places
                        result.t[i], result.y[i][0], result.y[i][1], result.y[i][2]
                    );
                }
            }
        }
        _ => {}
    }
}
