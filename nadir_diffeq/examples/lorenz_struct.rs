use nadir_diffeq::{
    OdeModel, OdeProblem,
    saving::{ResultStorage, SaveMethod},
    solvers::Solver,
    state::Adaptive,
    stepping::AdaptiveStepControl,
};
use std::{
    error::Error,
    ops::{AddAssign, MulAssign},
};
use tolerance::compute_error;

#[derive(Debug)]
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

impl Adaptive for LorenzState {
    fn compute_error(&self, x_prev: &Self, x_tilde: &Self, rel_tol: f64, abs_tol: f64) -> f64 {
        let mut sum_squared_errors = 0.0;
        // Calculate squared error for each component
        sum_squared_errors += compute_error(self.x, x_prev.x, x_tilde.x, rel_tol, abs_tol).powi(2);
        sum_squared_errors += compute_error(self.y, x_prev.y, x_tilde.y, rel_tol, abs_tol).powi(2);
        sum_squared_errors += compute_error(self.z, x_prev.z, x_tilde.z, rel_tol, abs_tol).powi(2);
        // Return RMS error
        (sum_squared_errors / 3.0).sqrt()
    }
}

impl OdeModel for Lorenz {
    type State = LorenzState;

    fn f(&mut self, _t: f64, x: &LorenzState, dx: &mut LorenzState) -> Result<(), Box<dyn Error>> {
        dx.x = self.sigma * (x.y - x.x);
        dx.y = x.x * (self.rho - x.z) - x.y;
        dx.z = x.x * x.y - self.beta * x.z;
        Ok(())
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let model = Lorenz {
        sigma: 10.,
        rho: 28.,
        beta: 8. / 3.,
    };

    let mut problem = OdeProblem::new(model);

    let x0 = LorenzState {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };

    let result = problem.solve_adaptive(
        &x0,
        (0.0, 1.0),
        AdaptiveStepControl::default(),
        Solver::DoPri45,
        SaveMethod::Memory,
    )?;

    match result {
        ResultStorage::Memory(result) => {
            for i in 0..result.t.len() {
                if result.t[i].rem_euclid(1.0) < 1e-3 {
                    println!(
                        "{:10.6}     {:10.6} {:10.6} {:10.6}",
                        result.t[i], result.y[i].x, result.y[i].y, result.y[i].z
                    );
                }
            }
        }
        _ => {}
    }
    Ok(())
}
