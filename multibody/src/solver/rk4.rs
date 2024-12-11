use aerospace::celestial_system::CelestialResult;
use spice::Spice;

use crate::{
    base::BaseSystems,
    joint::JointStates,
    result::{MultibodyResultTrait, ResultEntry},
    system::MultibodySystem,
    MultibodyErrors,
};
use std::{collections::HashMap, mem::take};

pub fn solve_fixed_rk4(
    sys: &mut MultibodySystem,
    tstart: f64,
    tstop: f64,
    mut dt: f64,
    spice: &mut Option<Spice>,
) -> Result<
    (
        Vec<f64>,
        HashMap<String, ResultEntry>,
        Option<CelestialResult>,
    ),
    MultibodyErrors,
> {
    if dt.abs() <= f64::EPSILON {
        return Err(MultibodyErrors::DtCantBeZero);
    };

    // Create a vec of JointStates as the initial state
    let x0 = JointStates(
        sys.joints
            .iter()
            .map(|joint| joint.borrow().model.state_vector_init())
            .collect(),
    );

    let mut half_dt = dt / 2.0;
    let mut dt_6 = dt / 6.0;
    let mut x = x0.clone();
    let mut t = tstart;

    let result_length = ((tstop - tstart) / dt).floor() as usize + 1;

    // initialize results

    let mut time = Vec::with_capacity(result_length);

    for body in &mut sys.bodies {
        body.borrow_mut().initialize_result(result_length);
    }

    for joint in &mut sys.joints {
        joint.borrow_mut().model.initialize_result(result_length);
    }

    for sensor in &mut sys.sensors {
        sensor.model.initialize_result(result_length);
    }

    // initialize celestial results
    match &mut sys.base.borrow_mut().system {
        BaseSystems::Basic(_) => {}
        BaseSystems::Celestial(celestial) => celestial.initialize_result(result_length),
    };

    // one time clone for initialization
    let mut k1 = x0.clone();
    let mut k2 = x0.clone();
    let mut k3 = x0.clone();
    let mut k4 = x0.clone();
    let mut tmp = x0.clone();

    //update body states based on initial joint states so that things like gravity can be calculated on first pass
    sys.update_body_states();

    for _ in 0..result_length {
        // calculate all secondary states with the current state
        // only joint states are required to integrate, but
        // we want to save things like body states, gravity , etc.
        sys.run(&mut tmp, &x, t, spice);

        // update the result vectors with the current values
        time.push(t);

        // update body results
        for body in &mut sys.bodies {
            body.borrow_mut().update_result();
        }

        // update joint results
        for joint in &mut sys.joints {
            joint.borrow_mut().model.update_result();
        }

        // update sensor results
        for sensor in &mut sys.sensors {
            sensor.model.update_result();
        }

        // update celestial results
        match &mut sys.base.borrow_mut().system {
            BaseSystems::Basic(_) => {} //nothing to do
            BaseSystems::Celestial(celestial) => {
                celestial.update_result(); // system.run() will update the values, including epoch
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

    // collect results
    let mut result_hm = HashMap::new();
    for body in &mut sys.bodies {
        let mut body = body.borrow_mut();
        result_hm.insert(body.name.clone(), body.get_result_entry());
    }
    for joint in &mut sys.joints {
        let mut joint = joint.borrow_mut();
        result_hm.insert(joint.name.clone(), joint.model.get_result_entry());
    }

    for sensor in &mut sys.sensors {
        result_hm.insert(sensor.name.clone(), sensor.model.get_result_entry());
    }

    let celestial = match &mut sys.base.borrow_mut().system {
        BaseSystems::Basic(_) => None,
        BaseSystems::Celestial(celestial) => {
            // add the celestial body states to the base result
            for body in &mut celestial.bodies {
                celestial.result.add_result(body);
            }
            let result = take(&mut celestial.result);
            Some(result)
        }
    };

    Ok((time, result_hm, celestial))
}
