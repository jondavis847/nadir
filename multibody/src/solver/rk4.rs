use std::sync::Arc;

use nadir_result::ResultManager;

use super::SimStates;
use crate::{system::MultibodySystem, MultibodyErrors};
use indicatif::{MultiProgress, ProgressBar, ProgressStyle};

pub fn solve_fixed_rk4(
    sys: &mut MultibodySystem,
    tstart: f64,
    tstop: f64,
    mut dt: f64,
    results: &mut ResultManager,
    progress_bars: Option<Arc<MultiProgress>>,
) -> Result<(), MultibodyErrors> {
    if dt.abs() <= f64::EPSILON {
        return Err(MultibodyErrors::DtCantBeZero);
    };

    // Create the vec of SimStates as the initial integration state
    let mut x0 = SimStates(Vec::new());
    for joint in &sys.joints {
        joint.borrow_mut().state_vector_init(&mut x0);
    }
    for actuator in &mut sys.actuators {
        actuator.state_vector_init(&mut x0);
    }

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
    let mut progress_bar = if let Some(mp) = progress_bars {
        let pb = mp.add(ProgressBar::new(result_length as u64));
        if let Some(run_id) = sys.run_id {
            pb.set_style(
                ProgressStyle::default_bar()
                    .template(&format!(
                        "Run {run_id:>3}: {{bar:40.cyan/black}} {{percent}}%"
                    ))
                    .unwrap()
                    .progress_chars("=> "),
            );
            Some(pb)
        } else {
            None
        }
    } else {
        let progress_bar = ProgressBar::new(result_length as u64);
        progress_bar.set_style(
            ProgressStyle::default_bar()
                .template("{spinner:.cyan} [{elapsed_precise}] {bar:40.cyan/black} {percent}%")
                .unwrap()
                .progress_chars("=> "),
        );
        Some(progress_bar)
    };

    for _ in 0..result_length {
        if let Some(progress_bar) = &mut progress_bar {
            progress_bar.inc(1);
        }

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
        sys.run(&mut k1, &tmp, t)?;
        //dbg!(&k1);

        // run software and update the results vectors here since we just updated all secondary states based on integrated state 'x'
        // NOTE: This may not be true for all solvers! This just saves us one function call
        // If not, call sys.run one more time before or after the stages to update
        // (or only call what's needed to update secondary states instead of sys.run)
        // add initial conditions to results storage
        sys.update_software()?;
        sys.write_result_files(t, results);

        // calculate k2 = f(x + 0.5*k1 , t + 0.5dt)
        tmp.clone_from(&k1);
        tmp *= half_dt;
        tmp += &x;
        sys.run(&mut k2, &tmp, t + half_dt)?;

        // calculate k3 = f(x + 0.5*k2 , t + 0.5dt)
        tmp.clone_from(&k2);
        tmp *= half_dt;
        tmp += &x;
        sys.run(&mut k3, &tmp, t + half_dt)?;

        // calculate k4 = f(x + k3 , t + dt)
        tmp.clone_from(&k3);
        tmp *= dt;
        tmp += &x;
        sys.run(&mut k4, &tmp, t + dt)?;

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

    results.flush();
    if let Some(progress_bar) = &mut progress_bar {
        progress_bar.finish();
    }

    Ok(())
}
