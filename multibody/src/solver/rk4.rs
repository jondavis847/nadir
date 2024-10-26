use spice::Spice;

use crate::{
    base::BaseSystems,
    body::BodyResult,
    joint::{joint_sim::JointSimTrait, joint_state::JointStates, JointResult},
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{SensorResult, SensorTrait},
    system_sim::MultibodySystemSim,
    MultibodyErrors, MultibodyTrait,
};
use std::collections::HashMap;

pub fn solve_fixed_rk4(
    sys: &mut MultibodySystemSim,
    tstart: f64,
    tstop: f64,
    mut dt: f64,
    spice: &mut Option<Spice>,
) -> Result<(Vec<f64>, HashMap<String, ResultEntry>), MultibodyErrors> {
    if dt.abs() <= f64::EPSILON {
        return Err(MultibodyErrors::DtCantBeZero);
    };

    // initialize body results
    let mut body_results: Vec<BodyResult> = sys
        .bodies
        .iter()
        .map(|body| body.initialize_result())
        .collect();

    // initialize joint results
    let mut joint_results: Vec<JointResult> = sys
        .joints
        .iter()
        .map(|joint| joint.initialize_result())
        .collect();

    // initialize sensor results
    let mut sensor_results: Vec<SensorResult> = sys
        .sensors
        .iter()
        .map(|(_, sensor)| sensor.initialize_result())
        .collect();

    // initialize celestial results
    let mut celestial_result = match &sys.base.system {
        BaseSystems::Basic(_) => None,
        BaseSystems::Celestial(celestial) => Some(celestial.initialize_result()),
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

    //update body states based on initial joint states so that things like gravity can be calculated on first pass
    sys.update_body_states();

    for i in 0..result_length {
        // calculate all secondary states with the current state
        // only joint states are required to integrate, but
        // we want to save things like body states, gravity , etc.
        sys.run(&mut tmp, &x, t, spice);

        // update the result vectors with the current values
        time[i] = t;

        // update body results
        sys.bodies
            .iter()
            .enumerate()
            .for_each(|(index, body)| body_results[index].update(body));

        // update joint results
        sys.joints
            .iter()
            .enumerate()
            .for_each(|(index, joint)| joint_results[index].update(joint));

        // update sensor results
        sys.sensors
            .iter()
            .enumerate()
            .for_each(|(index, (_, sensor))| sensor_results[index].update(sensor));

        // update celestial results
        if let Some(result) = &mut celestial_result {
            match &sys.base.system {
                BaseSystems::Basic(_) => {
                    unreachable!("we checked this when we created celestial_result")
                }
                BaseSystems::Celestial(celestial) => {
                    let current_epoch = celestial.epoch + t;
                    result.update(current_epoch,celestial)?
                },
            }
        }

        // change dt near end of sim to capture end point
        if (tstop - t) < dt && (tstop - t) > f64::EPSILON {
            dt = tstop - t;
            half_dt = dt / 2.0;
            dt_6 = dt / 6.0;
        }

        // calculate k1 = f(x,t)
        tmp.clone_from(&x);
        sys.run(&mut k1, &tmp, t, spice);

        // calculate k2 = f(x + 0.5*k1 , t + 0.5dt)
        tmp.clone_from(&k1);
        tmp *= half_dt;
        tmp += &x;
        sys.run(&mut k2, &tmp, t + half_dt, spice);

        // calculate k3 = f(x + 0.5*k2 , t + 0.5dt)
        tmp.clone_from(&k2);
        tmp *= half_dt;
        tmp += &x;
        sys.run(&mut k3, &tmp, t + half_dt, spice);

        // calculate k4 = f(x + k3 , t + dt)
        tmp.clone_from(&k3);
        tmp *= dt;
        tmp += &x;
        sys.run(&mut k4, &tmp, t + dt, spice);

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
    joint_results
        .iter()
        .enumerate()
        .for_each(|(index, result)| {
            result_hm.insert(sys.joint_names[index].clone(), result.get_result_entry());
        });

    body_results.iter().enumerate().for_each(|(index, result)| {
        result_hm.insert(sys.body_names[index].clone(), result.get_result_entry());
    });
    sys.sensors
        .iter()
        .enumerate()
        .for_each(|(index, (_, sensor))| {
            result_hm.insert(
                sensor.get_name().to_string(),
                sensor_results[index].get_result_entry(),
            );
        });
    if let Some(celestial) = celestial_result {
        result_hm.insert("celestial".to_string(),ResultEntry::Celestial(celestial));
    }
    
    Ok((time, result_hm))
}
