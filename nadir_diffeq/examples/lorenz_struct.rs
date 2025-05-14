use nadir_diffeq::{
    Integrable, OdeModel, OdeProblem, SaveMethod, Solver, StepMethod, StepPIDControl,
};
use std::ops::{AddAssign, MulAssign};
use tolerance::{Tolerance, Tolerances, compute_component_error};

struct Lorenz {
    sigma: f64,
    rho: f64,
    beta: f64,
}

#[derive(Clone, Default, Debug)]
struct LorenzState {
    x: f64,
    y: f64,
    z: f64,
}

impl AddAssign<&Self> for LorenzState {
    fn add_assign(&mut self, rhs: &Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl MulAssign<f64> for LorenzState {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl AddAssign<&LorenzDerivative> for LorenzState {
    fn add_assign(&mut self, rhs: &LorenzDerivative) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

#[derive(Clone, Default)]
struct LorenzDerivative {
    x: f64,
    y: f64,
    z: f64,
}

impl MulAssign<f64> for LorenzDerivative {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

struct LorenzTolerances {
    x: Option<Tolerances>,
    y: Option<Tolerances>,
    z: Option<Tolerances>,
}

impl Default for LorenzTolerances {
    fn default() -> Self {
        Self {
            x: None,
            y: None,
            z: Some(Tolerances::new(1e-5, 1e-7)),
        }
    }
}

impl Tolerance for LorenzTolerances {
    type State = LorenzState;

    fn compute_error(&self, x0: &Self::State, xf: &Self::State, rel_tol: f64, abs_tol: f64) -> f64 {
        let mut sum_squared_errors = 0.0;

        // Calculate squared error for each component
        sum_squared_errors += compute_component_error(&self.x, x0.x, xf.x, rel_tol, abs_tol);
        sum_squared_errors += compute_component_error(&self.y, x0.y, xf.y, rel_tol, abs_tol);
        sum_squared_errors += compute_component_error(&self.z, x0.z, xf.z, rel_tol, abs_tol);

        // Return RMS error
        (sum_squared_errors / 3.0).sqrt()
    }
}

impl Integrable for LorenzState {
    type Derivative = LorenzDerivative;
    type Tolerance = LorenzTolerances;

    fn initialize_writer(
        path: &std::path::PathBuf,
    ) -> Option<csv::Writer<std::io::BufWriter<std::fs::File>>> {
        // Create a new path by joining the directory path with the filename
        let file_path = path.join("result.csv");

        // Ensure the directory exists
        if let Some(parent) = file_path.parent() {
            if !parent.exists() {
                std::fs::create_dir_all(parent).unwrap();
            }
        }
        let file = std::fs::File::create(&file_path).unwrap();
        let mut writer = csv::Writer::from_writer(std::io::BufWriter::new(file));
        // Create header vector
        let headers = ["t", "x", "y", "z"];
        // Write headers
        writer.write_record(&headers).unwrap();
        // Flush to ensure headers are written
        writer.flush().unwrap();
        Some(writer)
    }

    fn save_to_writer(&self, writer: &mut csv::Writer<std::io::BufWriter<std::fs::File>>, t: f64) {
        // Using the serializer approach avoids string conversions
        let record = [t, self.x, self.y, self.z];

        // Serialize and write the record
        if let Err(err) = writer.serialize(&record) {
            eprintln!("Failed to serialize record at t={}: {}", t, err);
        }
    }
}

impl OdeModel<LorenzState> for Lorenz {
    fn f(&mut self, _t: f64, x: &LorenzState, dx: &mut LorenzDerivative) {
        dx.x = self.sigma * (x.y - x.x);
        dx.y = x.x * (self.rho - x.z) - x.y;
        dx.z = x.x * x.y - self.beta * x.z;
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
        Solver::New45,
        StepMethod::Adaptive(StepPIDControl::default()),
        SaveMethod::File(results_dir),
    );

    let x0 = LorenzState {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };

    solver.solve(&mut model, &x0, (0.0, 30.0));
}
