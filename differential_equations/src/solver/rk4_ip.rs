pub fn solve_fixed_rk4_in_place<F, P, T>(solver: &Solver<F, P, T>) -> (Vec<f64>, Vec<T>)
where
    F: OdeFunctionIP<P, T>,
    T: IntegrableIP,
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

    let half_dt = dt / 2.0;
    let x = &mut x0.clone();
    let mut t = *tstart;

    let result_length = ((tstop - tstart) / dt).ceil() as usize + 1;
    let mut result = Vec::with_capacity(result_length);
    result.extend(std::iter::repeat(x0.clone()).take(result_length));
    let mut time = Vec::with_capacity(result_length);
    time.extend(std::iter::repeat(0.0).take(result_length));
    let mut idx = 1 as usize; // 0 will be manually stored

    result[0].clone_from(x0);
    time[0].clone_from(&t);
    t += dt;

    //rk4 stages
    let k1 = &mut x0.clone();
    let k2 = &mut x0.clone();
    let k3 = &mut x0.clone();
    let k4 = &mut x0.clone();

    //tmp variables for in place stage calculation
    let k2_tmp = &mut x0.clone();
    let k3_tmp = &mut x0.clone();
    let k4_tmp = &mut x0.clone();
    let x2 = &mut x0.clone();
    let x3 = &mut x0.clone();
    let x4 = &mut x0.clone();

    while t <= *tstop {
        x2.clone_from(x);
        x3.clone_from(x);
        x4.clone_from(x);

        //k1
        func(k1, &x, parameters, t);
        //k2
        k2_tmp.clone_from(k1);
        *k2_tmp *= half_dt;
        *x2 += k2_tmp;
        func(k2, x2, parameters, t + half_dt);
        //k3
        k3_tmp.clone_from(k2);
        *k3_tmp *= half_dt;
        *x3 += k3_tmp;
        func(k3, x3, parameters, t + half_dt);
        //k4
        k4_tmp.clone_from(k3);
        *k4_tmp *= *dt;
        *x4 += k4_tmp;
        func(k4, x4, parameters, t + dt);

        //in place chaining for (k1 + 2k2 + 2k3 + k4) * dt/6

        *k2 *= 2.0;
        *k3 *= 2.0;
        *k1 += k2;
        *k1 += k3;
        *k1 += k4;
        *k1 *= dt / 6.0;
        *x += k1;

        result[idx].clone_from(x);
        time[idx].clone_from(&t);
        idx += 1;
        t += dt;
    }

    (time, result)
}
