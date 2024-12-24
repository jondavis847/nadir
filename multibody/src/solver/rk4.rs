use crate::{
    actuator::ActuatorSystem, joint::JointStates, sensor::SensorSystem, software::SoftwareSystem, system::{MultibodySystem, ResultWriters}, MultibodyErrors
};

pub fn solve_fixed_rk4<A,F,S>(
    sys: &mut MultibodySystem<A,F,S>,
    tstart: f64,
    tstop: f64,
    mut dt: f64,
    writers: &mut ResultWriters,
) -> Result<(), MultibodyErrors> 
where 
A: ActuatorSystem, 
F: SoftwareSystem, 
S: SensorSystem{
    if dt.abs() <= f64::EPSILON {
        return Err(MultibodyErrors::DtCantBeZero);
    };

    // Create a vec of JointStates as the initial integration state
    let x0 = JointStates(
        sys.joints
            .iter()
            .map(|joint| joint.borrow().model.state_vector_init())
            .collect(),
    );

    // define some variables
    let mut half_dt = dt / 2.0;
    let mut dt_6 = dt / 6.0;
    let mut x = x0.clone();
    let mut t = tstart;

    // one time clone for preallocation of temporary variables
    let mut k1 = x0.clone();
    let mut k2 = x0.clone();
    let mut k3 = x0.clone();
    let mut k4 = x0.clone();
    let mut tmp = x0.clone();

    let result_length = ((tstop - tstart) / dt).floor() as usize + 1;

    // RUN THE RK4 LOOP
    for _ in 0..result_length {
        // update sensors
        sys.sensors.iter_mut().for_each(|sensor| sensor.update());

        // logic to change dt near end of sim to capture end point
        if (tstop - t) < dt && (tstop - t) > f64::EPSILON {
            dt = tstop - t;
            half_dt = dt / 2.0;
            dt_6 = dt / 6.0;
        }

        // BEGIN RK4 STAGES
        //println!("k1");
        // calculate k1 = f(x,t)
        tmp.clone_from(&x);
        sys.run(&mut k1, &tmp, t);
        //dbg!(&k1);

        // update the results vectors here since we just updated all secondary states based on integrated state 'x'
        // NOTE: This may not be true for all solvers! This just saves us one function call
        // If not, call sys.run one more time before or after the stages to update
        // (or only call what's needed to update secondary states instead of sys.run)
        // add initial conditions to results storage
        sys.write_result_files(t, writers);

        //println!("k2");
        // calculate k2 = f(x + 0.5*k1 , t + 0.5dt)
        tmp.clone_from(&k1);
        tmp *= half_dt;
        tmp += &x;
        sys.run(&mut k2, &tmp, t + half_dt);

        //println!("k3");
        // calculate k3 = f(x + 0.5*k2 , t + 0.5dt)
        tmp.clone_from(&k2);
        tmp *= half_dt;
        tmp += &x;
        sys.run(&mut k3, &tmp, t + half_dt);

        //println!("k4");
        // calculate k4 = f(x + k3 , t + dt)
        tmp.clone_from(&k3);
        tmp *= dt;
        tmp += &x;
        sys.run(&mut k4, &tmp, t + dt);

        // calculate x = x + (k1 + k2 * 2.0 + k3 * 2.0 + k4) * dt / 6.0;

        k1 *= dt_6;
        x += &k1;

        k2 *= 2.0 * dt_6;
        x += &k2;

        k3 *= 2.0 * dt_6;
        x += &k3;

        k4 *= dt_6;
        x += &k4;

        //dbg!(&x);

        t += dt;
    }
    // flush the writers
    writers.flush();

    Ok(())
}
