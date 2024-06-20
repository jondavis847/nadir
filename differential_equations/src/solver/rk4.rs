use super::{Integrable, Solver};

pub fn solve_fixed_rk4<F, T>(solver: &Solver<F, T>) -> (Vec<f64>, Vec<T>)
where
    F: Fn(T, f64) -> T,
    T: Integrable,
{
    let Solver {
        func,
        x0,
        tstart,
        tstop,
        dt,
        ..
    } = solver;

    assert!(dt.abs() > f64::EPSILON, "0.0 dt not allowed!");

    let half_dt = dt / 2.0;
    let mut x = *x0;
    let mut t = *tstart;

    let result_length = ((tstop - tstart) / dt).ceil() as usize;
    let mut result = Vec::with_capacity(result_length);
    let mut time = Vec::with_capacity(result_length);

    result.push(x);
    time.push(t);

    while t <= *tstop {
        let k1 = func(x, t);
        let k2 = func(x + k1 * half_dt, t + half_dt);
        let k3 = func(x + k2 * half_dt, t + half_dt);
        let k4 = func(x + k3 * *dt, t + dt);

        x = x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * *dt / 6.0;
        t += dt;

        result.push(x);
        time.push(t);
    }

    (time, result)
}
