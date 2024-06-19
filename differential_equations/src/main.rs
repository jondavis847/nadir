use differential_equations::{
    solver::{rk4::Rk4, SolverTrait},
    Integrable,
};
use std::ops::{Add, Div, Mul};

// PendulumState represents the state of the pendulum with angle `theta` and angular velocity `omega`.
#[derive(Debug, Clone, Copy)]
struct PendulumState {
    theta: f64,
    omega: f64,
}

impl Add for PendulumState {
    type Output = Self;

    fn add(self, other: Self) -> Self {
        Self {
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

impl Integrable for PendulumState {}

// Define the equations of motion for the pendulum.
fn pendulum_dynamics(state: PendulumState, _t: f64) -> PendulumState {
    let g = 9.81; // Acceleration due to gravity (m/s^2)
    let l = 1.0; // Length of the pendulum (m)

    PendulumState {
        theta: state.omega,
        omega: -(g / l) * state.theta.sin(),
    }
}

fn main() {
    let initial_state = PendulumState {
        theta: 1.0,
        omega: 0.0,
    }; // Initial state of the pendulum
    let tspan = (0.0, 10.0); // Time span for the simulation
    let dt = 0.1; // Time step (10 Hz)

    let mut solver = Rk4 {};
    let (time, results) = solver.solve_fixed(initial_state, pendulum_dynamics, tspan, dt);

    // Print the results
    for (i, state) in results.iter().enumerate() {
        println!(
            "Time: {:.2}, Theta: {:.4}, Omega: {:.4}",
            time[i], state.theta, state.omega
        );
    }
}
