use nadir_diffeq::{
    OdeModel, OdeProblem, Solver,
    saving::{ResultStorage, SaveMethod},
    state::{Integrable, StateWriterBuilder},
    stepping::{AdaptiveStepControl, StepMethod},
};
use std::{
    error::Error,
    fmt::Write,
    ops::{AddAssign, MulAssign},
    path::PathBuf,
};
use tolerance::{Tolerance, Tolerances, compute_component_error};

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

impl AddAssign<&LorenzDerivative> for LorenzState {
    fn add_assign(&mut self, rhs: &LorenzDerivative) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
    }
}

#[derive(Clone, Default, Debug)]
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

    fn compute_error(
        &self,
        x: &Self::State,
        x_prev: &Self::State,
        x_tilde: &Self::State,
        rel_tol: f64,
        abs_tol: f64,
    ) -> f64 {
        let mut sum_squared_errors = 0.0;

        // Calculate squared error for each component
        sum_squared_errors +=
            compute_component_error(&self.x, x.x, x_prev.x, x_tilde.x, rel_tol, abs_tol).powi(2);
        sum_squared_errors +=
            compute_component_error(&self.y, x.y, x_prev.y, x_tilde.y, rel_tol, abs_tol).powi(2);
        sum_squared_errors +=
            compute_component_error(&self.z, x.z, x_prev.z, x_tilde.z, rel_tol, abs_tol).powi(2);

        // Return RMS error
        (sum_squared_errors / 3.0).sqrt()
    }
}

impl Integrable for LorenzState {
    type Derivative = LorenzDerivative;
    type Tolerance = LorenzTolerances;

    fn writer(path: PathBuf) -> StateWriterBuilder<Self> {
        StateWriterBuilder::new(path, |t, x: &Self, buffer: &mut Vec<String>| {
            if buffer.len() != 4 {
                buffer.resize(4, String::new());
            }
            write!(buffer[0], "{}", t)?;
            write!(buffer[1], "{}", x.x)?;
            write!(buffer[2], "{}", x.y)?;
            write!(buffer[3], "{}", x.z)?;
            Ok(())
        })
    }
}

impl OdeModel<LorenzState> for Lorenz {
    fn f(
        &mut self,
        _t: f64,
        x: &LorenzState,
        dx: &mut LorenzDerivative,
    ) -> Result<(), Box<dyn Error>> {
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

    let mut problem = OdeProblem::new(
        model,
        Solver::DoPri45,
        StepMethod::Adaptive(
            AdaptiveStepControl::default()
                .with_rel_tol(1e-9)
                .with_abs_tol(1e-6),
        ),
        SaveMethod::Memory,
    );

    let x0 = LorenzState {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };

    let result = problem.solve(&x0, (0.0, 1.0))?;

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
