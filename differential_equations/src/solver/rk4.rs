use crate::{solver::Solver, Integrable, OdeFunction};

pub fn solve_fixed_rk4<F, P, T>(solver: &mut Solver<F, P, T>) -> (Vec<f64>, Vec<T>)
where
    F: OdeFunction<P, T>,
    T: Integrable,
{
    let Solver {
        func,
        x0,
        parameters,
        tstart,
        tstop,
        dt,
        ..
    } = solver;

    assert!(dt.abs() > f64::EPSILON, "0.0 dt not allowed!");

    let mut half_dt = *dt / 2.0;
    let mut dt_6 = *dt / 6.0;
    let mut x = x0.clone();
    let mut t = *tstart;

    // using resize instead of with_capacity so we can clone_from results instead of .push(clone())
    let result_length = ((*tstop - *tstart) / *dt).floor() as usize + 1;
    let mut result = Vec::new();
    result.resize(result_length, x0.clone());

    let mut time = Vec::new();
    time.resize(result_length, 0.0);

    result[0].clone_from(&x0);
    time[0].clone_from(&t);

    // one time clone for initialization
    let mut k1 = x0.clone();
    let mut k2 = x0.clone();
    let mut k3 = x0.clone();
    let mut k4 = x0.clone();
    let mut tmp = x0.clone();    

    for i in 1..result_length {
        // change dt near end of sim to capture end point
        if (*tstop - t) < *dt && (*tstop - t) > f64::EPSILON {
            *dt = *tstop - t;
            half_dt = *dt / 2.0;
            dt_6 = *dt / 6.0;
        }

        // calculate k1 = f(x,t)
        tmp.clone_from(&x);
        func(&mut k1, &tmp, parameters, t);

        // calculate k2 = f(x + 0.5*k1 , t + 0.5*dt)
        tmp.clone_from(&k1);
        tmp *= half_dt;
        tmp += &x;
        func(&mut k2, &tmp, parameters, t + half_dt);

        // calculate k3 = f(x + 0.5*k2 , t + 0.5*dt)
        tmp.clone_from(&k2);
        tmp *= half_dt;
        tmp += &x;
        func(&mut k3, &tmp, parameters, t + half_dt);

        // calculate k4 = f(x + k3 , t + dt)
        tmp.clone_from(&k3);        
        tmp *= *dt;
        tmp += &x;
        func(&mut k4, &tmp, parameters, t + *dt);

        // calculate x = x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * dt / 6.0;
        
        k1 *= dt_6;
        x += &k1;

        k2 *= 2.0 * dt_6;
        x += &k2;

        k3 *= 2.0 * dt_6;
        x += &k3;

        k4 *= dt_6;
        x += &k4;

        t += *dt;

        // call any callbacks for the new state now
        for cb in solver.callbacks.iter_mut() {
            cb(&mut x, parameters, &mut t);
        }

        result[i].clone_from(&x);
        time[i].clone_from(&t);
    }

    (time, result)
}
