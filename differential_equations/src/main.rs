use differential_equations::solver::{Solver, SolverMethod};
use std::ops::{Add, Div, Mul};

// PendulumState represents the state of the pendulum with angle `theta` and angular velocity `omega`.
#[derive(Debug, Clone, Copy)]
struct PendulumState {
    theta: f64,
    omega: f64,
}

impl Add<Self> for PendulumState {
    type Output = Self;
    fn add(self, other: Self) -> Self {
        PendulumState {
            theta: self.theta + other.theta,
            omega: self.omega + other.omega,
        }
    }
}

impl Mul<f64> for PendulumState {
    type Output = Self;
    fn mul(self, rhs: f64) -> Self {
        Self {
            theta: self.theta * rhs,
            omega: self.omega * rhs,
        }
    }
}

impl Div<f64> for PendulumState {
    type Output = Self;
    fn div(self, rhs: f64) -> Self {
        Self {
            theta: self.theta / rhs,
            omega: self.omega / rhs,
        }
    }
}

struct PendulumParameters {
    pub g: f64,
    pub l: f64,
}

// Define the equations of motion for the pendulum.
fn pendulum_dynamics(x: &PendulumState, p: &Option<PendulumParameters>, _t: f64) -> PendulumState {
    let p = p.as_ref().unwrap();

    PendulumState {
        theta: x.omega,
        omega: -(p.g / p.l) * x.theta.sin(),
    }
}

fn main() {
    let initial_state = PendulumState {
        theta: 1.0,
        omega: 0.0,
    }; // Initial state of the pendulum

    let p = PendulumParameters { g: 9.81, l: 1.0 };

    let mut solver = Solver {
        func: |x, p, t| pendulum_dynamics(x, p, t),
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
