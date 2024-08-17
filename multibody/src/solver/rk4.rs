use crate::{    
    joint::{
        joint_sim::{JointSim, JointSimTrait},
        joint_state::JointStates,        
        JointResult,
    },
    result::{update_body_states, ResultEntry},
    system_sim::MultibodySystemSim,
    MultibodyErrors,
};
use std::collections::HashMap;

pub fn solve_fixed_rk4(
    sys: &mut MultibodySystemSim,
    tstart: f64,
    tstop: f64,
    mut dt: f64,
) -> Result<(Vec<f64>, HashMap<String, ResultEntry>), MultibodyErrors> {
    if dt.abs() <= f64::EPSILON {
        return Err(MultibodyErrors::DtCantBeZero);
    };

    // Create a vec of JointStates as the initial state
    let x0 = JointStates(sys.joints.iter().map(|joint| joint.get_state()).collect());

    let mut half_dt = dt / 2.0;
    let mut dt_6 = dt / 6.0;
    let mut x = x0.clone();
    let mut t = tstart;

    // using resize instead of with_capacity so we can clone_from results instead of .push(clone())
    let result_length = ((tstop - tstart) / dt).floor() as usize + 1;

    let mut time = Vec::new();
    time.resize(result_length, 0.0);

    // one time clone for initialization
    let mut k1 = x0.clone();
    let mut k2 = x0.clone();
    let mut k3 = x0.clone();
    let mut k4 = x0.clone();
    let mut tmp = x0.clone();

    for i in 0..result_length {
        // calculate all secondary states with the current state
        // i.e. only joint states are required to integrate, but
        // we want to save things like body states, gravity , etc.
        sys.run(&mut tmp, &x, t);
        update_body_states(&mut sys.bodies, &sys.joints);

        // update the result vectors with the current values
        time[i] = t;
        sys.joints.iter_mut().for_each(|joint| joint.set_result());
        sys.bodies.iter_mut().for_each(|body| body.set_result());

        // change dt near end of sim to capture end point
        if (tstop - t) < dt && (tstop - t) > f64::EPSILON {
            dt = tstop - t;
            half_dt = dt / 2.0;
            dt_6 = dt / 6.0;
        }

        // calculate k1 = f(x,t)
        tmp.clone_from(&x);
        sys.run(&mut k1, &tmp, t);

        // calculate k2 = f(x + 0.5*k1 , t + 0.5dt)
        tmp.clone_from(&k1);
        tmp *= half_dt;
        tmp += &x;
        sys.run(&mut k2, &tmp, t + half_dt);

        // calculate k3 = f(x + 0.5*k2 , t + 0.5dt)
        tmp.clone_from(&k2);
        tmp *= half_dt;
        tmp += &x;
        sys.run(&mut k3, &tmp, t + half_dt);

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

        t += dt;
    }

    let mut result_hm = HashMap::<String, ResultEntry>::new();
    sys.joints
        .iter_mut()
        .enumerate()
        .for_each(|(i, joint)| match joint {
            JointSim::Revolute(revolute) => {
                result_hm.insert(
                    sys.joint_names[i].clone(),
                    ResultEntry::Joint(JointResult::Revolute(std::mem::take(&mut revolute.result))),
                );
            }
            JointSim::Prismatic(prismatic) => {
                result_hm.insert(
                    sys.joint_names[i].clone(),
                    ResultEntry::Joint(JointResult::Prismatic(std::mem::take(
                        &mut prismatic.result,
                    ))),
                );
            }
        });
    sys.bodies.iter_mut().enumerate().for_each(|(i, body)| {
        result_hm.insert(
            sys.body_names[i].clone(),
            ResultEntry::Body(std::mem::take(&mut body.result)),
        );
    });

    Ok((time, result_hm))
}
