use criterion::{black_box, criterion_group, criterion_main, Criterion};
use differential_equations::{Integrable,solver::{Solver, SolverMethod}};
use std::ops::{AddAssign, MulAssign};

// PendulumState represents the state of the pendulum with angle `theta` and angular velocity `omega`.
#[derive(Debug, Clone, Copy)]
struct PendulumState {
    pub theta: f64,
    pub omega: f64,
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

fn run_simulation() {
    let initial_state = PendulumState {
        theta: 1.0,
        omega: 0.0,
    }; // Initial state of the pendulum

    let p = PendulumParameters { g: 9.81, l: 1.0 };

    let mut solver = Solver {
        func: |dx, x, p, t| pendulum_dynamics(dx, x, p, t),
        x0: initial_state,
        parameters: Some(p),
        tstart: 0.0,
        tstop: 1000.0,
        dt: 0.1,
        solver: SolverMethod::Rk4Classical,
        callbacks: Vec::new(),
    };

    solver.solve();
}

fn criterion_benchmark(c: &mut Criterion) {
    c.bench_function("pendulum_simulation", |b| {
        b.iter(|| black_box(run_simulation()))
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);
