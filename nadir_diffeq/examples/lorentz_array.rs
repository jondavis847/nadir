use nadir_diffeq::{OdeModel, OdeSolver, SaveMethod, Solver, StepMethod, state_array::StateArray};

struct Lorentz {
    sigma: f64,
    rho: f64,
    beta: f64,
}

impl OdeModel<StateArray<3>> for Lorentz {
    fn f(&mut self, _t: f64, x: &StateArray<3>, dx: &mut StateArray<3>) {
        dx[0] = self.sigma * (x[1] - x[0]);
        dx[1] = x[0] * (self.rho - x[2]) - x[1];
        dx[2] = x[0] * x[1] - self.beta * x[2];
    }
}

fn main() {
    let mut model = Lorentz {
        sigma: 10.,
        rho: 28.,
        beta: 3. / 8.,
    };
    let mut solver = OdeSolver::new(Solver::Rk4, StepMethod::Fixed(0.001), SaveMethod::Memory);

    let x0 = StateArray::new([1.0, 0.0, 0.0]); // Initial conditions for x, y, z{

    let result = solver.solve(&mut model, &x0, (0.0, 30.0));
    if let Some(result) = result {
        for i in 0..result.t.len() {
            if result.t[i] - result.t[i].floor() < 1e-4 {
                println!(
                    "{:10.6}     {:10.6}     {:10.6}     {:10.6}", // 10 chars wide, 6 decimal places
                    result.t[i], result.y[i][0], result.y[i][1], result.y[i][2]
                );
            }
        }
    }
}
