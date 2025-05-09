use nadir_diffeq::{Integrable, OdeModel, OdeSolver, SaveMethod, Solver, StepMethod};
use std::ops::{AddAssign, MulAssign};

struct Lorentz {
    sigma: f64,
    rho: f64,
    beta: f64,
}

#[derive(Clone, Default)]
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

impl Integrable for LorentzState {
    type Derivative = LorentzDerivative;
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

    let mut solver = OdeSolver::new(
        Solver::DoPri45,
        StepMethod::Fixed(0.001),
        SaveMethod::Memory,
    );

    let x0 = LorentzState {
        x: 1.0,
        y: 0.0,
        z: 0.0,
    };

    let result = solver.solve(&mut model, &x0, (0.0, 30.0));
    if let Some(result) = result {
        for i in 0..result.t.len() {
            println!(
                "{:10.6}     {:10.6}     {:10.6}     {:10.6}", // 10 chars wide, 6 decimal places
                result.t[i], result.y[i].x, result.y[i].y, result.y[i].z
            );
        }
    }
}
