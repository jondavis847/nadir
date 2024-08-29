use differential_equations::solver::{Solver, SolverMethod};
use std::ops::{AddAssign, MulAssign};

// PendulumState represents the state of the pendulum with angle `theta` and angular velocity `omega`.
#[derive(Debug, Clone, Copy)]
struct PendulumState {
    theta: f64,
    omega: f64,
}

impl<'a> AddAssign<&'a Self> for PendulumState {    
    fn add_assign(&mut self, rhs: &'a Self) {
        self.theta += rhs.theta;
        self.omega += rhs.omega;        
    }
}

impl MulAssign<f64> for PendulumState {    
    fn mul_assign(&mut self, rhs: f64) {
        self.theta *= rhs;
        self.omega *= rhs;
    }
}

struct PendulumParameters {
    pub g: f64,
    pub l: f64,
}

// Define the equations of motion for the pendulum.
fn pendulum_dynamics(dx: &mut PendulumState, x: &PendulumState, p: &Option<PendulumParameters>, _t: f64) {
    let p = p.as_ref().unwrap();

    dx.theta = x.omega;
    dx.omega = -(p.g / p.l) * x.theta.sin();    
}

fn main() {
    let initial_state = PendulumState {
        theta: 1.0,
        omega: 0.0,
    }; // Initial state of the pendulum

    let p = PendulumParameters { g: 9.81, l: 1.0 };

    let mut solver = Solver {
        func: |dx,x, p, t| pendulum_dynamics(dx, x, p, t),
        x0: initial_state,
        parameters: Some(p),
        tstart: 0.0,
        tstop: 10.0,
        dt: 0.1,
        solver: SolverMethod::Rk4Classical,
        callbacks: Vec::new(),
    };

    let (time, results) = solver.solve();

    // Print the results
    for (i, state) in results.iter().enumerate() {
        println!(
            "Time: {:.2}, Theta: {:.4}, Omega: {:.4}",
            time[i], state.theta, state.omega
        );
    }
}
