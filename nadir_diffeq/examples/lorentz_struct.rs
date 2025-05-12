use nadir_diffeq::{
    Integrable, OdeModel, OdeProblem, SaveMethod, Solver, StepMethod, result::ResultStorage,
};
use std::ops::{AddAssign, MulAssign};
use tolerance::{Tolerance, Tolerances, compute_error};

struct Lorentz {
    sigma: f64,
    rho: f64,
    beta: f64,
}

#[derive(Clone, Default, Debug)]
struct LorentzState {
    x: f64,
    y: f64,
    z: f64,
}

impl AddAssign<&Self> for LorentzState {
    fn add_assign(&mut self, rhs: &Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

impl MulAssign<f64> for LorentzState {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

impl AddAssign<&LorentzDerivative> for LorentzState {
    fn add_assign(&mut self, rhs: &LorentzDerivative) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

#[derive(Clone, Default)]
struct LorentzDerivative {
    x: f64,
    y: f64,
    z: f64,
}

impl MulAssign<f64> for LorentzDerivative {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
    }
}

struct LorentzTolerances {
    x: Option<Tolerances>,
    y: Option<Tolerances>,
    z: Option<Tolerances>,
}

impl Default for LorentzTolerances {
    fn default() -> Self {
        Self {
            x: None,
            y: None,
            z: Some(Tolerances::new(1e-5, 1e-7)),
        }
    }
}

impl Tolerance for LorentzTolerances {
    type State = LorentzState;

    fn compute_error(&self, x0: &Self::State, xf: &Self::State, rel_tol: f64, abs_tol: f64) -> f64 {
        let mut sum_squared_errors = 0.0;

        // Calculate squared error for x component
        if let Some(tol) = self.x {
            let err = tol.compute_error(x0.x, xf.x);
            sum_squared_errors += err * err;
        } else {
            let err = compute_error(x0.x, xf.x, rel_tol, abs_tol);
            sum_squared_errors += err * err;
        }

        // Calculate squared error for y component
        if let Some(tol) = self.y {
            let err = tol.compute_error(x0.y, xf.y);
            sum_squared_errors += err * err;
        } else {
            let err = compute_error(x0.y, xf.y, rel_tol, abs_tol);
            sum_squared_errors += err * err;
        }

        // Calculate squared error for z component
        if let Some(tol) = self.z {
            let err = tol.compute_error(x0.z, xf.z);
            sum_squared_errors += err * err;
        } else {
            let err = compute_error(x0.z, xf.z, rel_tol, abs_tol);
            sum_squared_errors += err * err;
        }

        // Return RMS error
        (sum_squared_errors / 3.0).sqrt()
    }
}

impl Integrable for LorentzState {
    type Derivative = LorentzDerivative;
    type Tolerance = LorentzTolerances;
}

impl OdeModel<LorentzState> for Lorentz {
    fn f(&mut self, _t: f64, x: &LorentzState, dx: &mut LorentzDerivative) {
        dx.x = self.sigma * (x.y - x.x);
        dx.y = x.x * (self.rho - x.z) - x.y;
        dx.z = x.x * x.y - self.beta * x.z;
    }
}

fn main() {
    let mut model = Lorentz {
        sigma: 10.,
        rho: 28.,
        beta: 3. / 8.,
    };

    let mut solver = OdeProblem::new(
        Solver::DoPri45,
        StepMethod::Adaptive {
            rel_tol: 1e-6,
            abs_tol: 1e-9,
            max_dt: None,
            min_dt: None,
        },
        SaveMethod::Memory,
    );

    let x0 = LorentzState {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };

    let result = solver.solve(&mut model, &x0, (0.0, 1000.0));

    match result {
        ResultStorage::Memory(result) => {
            for i in 0..result.t.len() {
                if result.t[i] - result.t[i].floor() < 1e-3 {
                    println!(
                        "{:10.6}     {:10.6}     {:10.6}     {:10.6}", // 10 chars wide, 6 decimal places
                        result.t[i], result.y[i].x, result.y[i].y, result.y[i].z
                    );
                }
            }
        }
        _ => {}
    }
}
