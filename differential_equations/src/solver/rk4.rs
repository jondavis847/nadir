use super::{Integrable, SolverTrait};

pub struct Rk4 {}
// just needed if we ever decide to implement better tableaus
#[derive(Debug, Clone)]
struct Rk4ButcherTableau {
    a: [[f64; 4]; 4], // Coefficients for the stages
    b: [f64; 4],      // Weights for the final combination
    c: [f64; 4],      // Nodes in the time step
}

impl Rk4ButcherTableau {
    fn new() -> Self {
        Rk4ButcherTableau {
            a: [
                [0.0, 0.0, 0.0, 0.0],
                [0.5, 0.0, 0.0, 0.0],
                [0.0, 0.5, 0.0, 0.0],
                [0.0, 0.0, 1.0, 0.0],
            ],
            b: [1.0 / 6.0, 1.0 / 3.0, 1.0 / 3.0, 1.0 / 6.0],
            c: [0.0, 0.5, 0.5, 1.0],
        }
    }
}

impl SolverTrait for Rk4 {
    fn solve_fixed<T, F>(&mut self, x0: T, func: F, tspan: (f64, f64), dt: f64) -> (Vec<f64>,Vec<T>)
    where
        T: Integrable,
        F: Fn(T, f64) -> T,
    {
        assert!(dt.abs() > f64::EPSILON, "0.0 dt not allowed!");

        let half_dt = dt / 2.0;
        let mut x = x0;
        let mut t = tspan.0;

        
        let result_length = ((tspan.1 - tspan.0) / dt).ceil() as usize;
        let mut result = Vec::<T>::with_capacity(result_length);
        result.push(x);
        let mut time = Vec::<f64>::with_capacity(result_length);
        time.push(t);

        while t <= tspan.1 {
            let k1 = func(x, t);
            let k2 = func(x + k1 * half_dt, t + half_dt);
            let k3 = func(x + k2 * half_dt, t + half_dt);
            let k4 = func(x + k3 * dt, t + dt);

            x = x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * dt / 6.0;
            t += dt;

            result.push(x.clone());
            time.push(t);
        }
        (time,result)
    }
}
