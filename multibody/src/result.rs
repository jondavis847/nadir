use chrono::{DateTime, Utc};
use rotations::quaternion::Quaternion;
use std::collections::{HashMap, HashSet};
use std::fmt;
use std::time::{Duration, SystemTime};
use utilities::format_duration;

use polars::prelude::*;

use crate::{
    algorithms::articulated_body_algorithm::ArticulatedBodyAlgorithm,
    body::{BodyResult, BodySim},
    joint::{JointResult, JointSim, JointSimTrait},
    system_sim::MultibodySystemSim,
};

pub struct MultibodyResult {
    pub name: String,
    pub result: HashMap<String, ResultEntry>,
    pub system: MultibodySystemSim,
    pub time_start: SystemTime,
    pub sim_duration: Duration,
    pub total_duration: Duration,
}

impl MultibodyResult {
    pub fn get_component(&self, component_name: &str) -> DataFrame {
        let component = self.result.get(component_name).unwrap();
        let mut df = DataFrame::default();

        let t = match self.result.get("t") {
            Some(ResultEntry::VecF64(vec)) => Series::new("t", vec.clone()),
            _ => panic!("Could not find `t`, this should not be possible"),
        };
        df.with_column(t).unwrap();

        match component {
            ResultEntry::Joint(joint) => match joint {
                JointResult::Revolute(revolute) => {
                    let theta = Series::new("theta", revolute.theta.clone());
                    let omega = Series::new("omega", revolute.omega.clone());
                    df.with_column(theta).unwrap();
                    df.with_column(omega).unwrap();
                }
                JointResult::Prismatic(prismatic) => {
                    let position = Series::new("position", prismatic.position.clone());
                    let velocity = Series::new("velocity", prismatic.velocity.clone());
                    df.with_column(position).unwrap();
                    df.with_column(velocity).unwrap();
                } //_ => panic!("Invalid joint type"),
            },
            ResultEntry::Body(body) => {
                let position_base_x = Series::new(
                    "position_base_x",
                    body.position_base
                        .iter()
                        .map(|v| v.e1)
                        .collect::<Vec<f64>>(),
                );
                let position_base_y = Series::new(
                    "position_base_y",
                    body.position_base.iter().map(|v| v.e2).collect::<Vec<_>>(),
                );
                let position_base_z = Series::new(
                    "position_base_z",
                    body.position_base.iter().map(|v| v.e3).collect::<Vec<_>>(),
                );

                let velocity_base_x = Series::new(
                    "velocity_base_x",
                    body.velocity_base.iter().map(|v| v.e1).collect::<Vec<_>>(),
                );
                let velocity_base_y = Series::new(
                    "velocity_base_y",
                    body.velocity_base.iter().map(|v| v.e2).collect::<Vec<_>>(),
                );
                let velocity_base_z = Series::new(
                    "velocity_base_z",
                    body.velocity_base.iter().map(|v| v.e3).collect::<Vec<_>>(),
                );

                let acceleration_base_x = Series::new(
                    "acceleration_base_x",
                    body.acceleration_base
                        .iter()
                        .map(|v| v.e1)
                        .collect::<Vec<_>>(),
                );
                let acceleration_base_y = Series::new(
                    "acceleration_base_y",
                    body.acceleration_base
                        .iter()
                        .map(|v| v.e2)
                        .collect::<Vec<_>>(),
                );
                let acceleration_base_z = Series::new(
                    "acceleration_base_z",
                    body.acceleration_base
                        .iter()
                        .map(|v| v.e3)
                        .collect::<Vec<_>>(),
                );

                let acceleration_body_x = Series::new(
                    "acceleration_body_x",
                    body.acceleration_body
                        .iter()
                        .map(|v| v.e1)
                        .collect::<Vec<_>>(),
                );
                let acceleration_body_y = Series::new(
                    "acceleration_body_y",
                    body.acceleration_body
                        .iter()
                        .map(|v| v.e2)
                        .collect::<Vec<_>>(),
                );
                let acceleration_body_z = Series::new(
                    "acceleration_body_z",
                    body.acceleration_body
                        .iter()
                        .map(|v| v.e3)
                        .collect::<Vec<_>>(),
                );

                let angular_accel_body_x = Series::new(
                    "angular_accel_body_x",
                    body.angular_accel_body
                        .iter()
                        .map(|v| v.e1)
                        .collect::<Vec<_>>(),
                );
                let angular_accel_body_y = Series::new(
                    "angular_accel_body_y",
                    body.angular_accel_body
                        .iter()
                        .map(|v| v.e2)
                        .collect::<Vec<_>>(),
                );
                let angular_accel_body_z = Series::new(
                    "angular_accel_body_z",
                    body.angular_accel_body
                        .iter()
                        .map(|v| v.e3)
                        .collect::<Vec<_>>(),
                );

                let angular_rate_body_x = Series::new(
                    "angular_rate_body_x",
                    body.angular_rate_body
                        .iter()
                        .map(|v| v.e1)
                        .collect::<Vec<_>>(),
                );
                let angular_rate_body_y = Series::new(
                    "angular_rate_body_y",
                    body.angular_rate_body
                        .iter()
                        .map(|v| v.e2)
                        .collect::<Vec<_>>(),
                );
                let angular_rate_body_z = Series::new(
                    "angular_rate_body_z",
                    body.angular_rate_body
                        .iter()
                        .map(|v| v.e3)
                        .collect::<Vec<_>>(),
                );

                let attitude_base_s = Series::new(
                    "attitude_base_s",
                    body.attitude_base.iter().map(|q| q.s).collect::<Vec<_>>(),
                );
                let attitude_base_x = Series::new(
                    "attitude_base_x",
                    body.attitude_base.iter().map(|q| q.x).collect::<Vec<_>>(),
                );
                let attitude_base_y = Series::new(
                    "attitude_base_y",
                    body.attitude_base.iter().map(|q| q.y).collect::<Vec<_>>(),
                );
                let attitude_base_z = Series::new(
                    "attitude_base_z",
                    body.attitude_base.iter().map(|q| q.z).collect::<Vec<_>>(),
                );

                let external_force_body_x = Series::new(
                    "external_force_body_x",
                    body.external_force_body
                        .iter()
                        .map(|v| v.e1)
                        .collect::<Vec<_>>(),
                );
                let external_force_body_y = Series::new(
                    "external_force_body_y",
                    body.external_force_body
                        .iter()
                        .map(|v| v.e2)
                        .collect::<Vec<_>>(),
                );
                let external_force_body_z = Series::new(
                    "external_force_body_z",
                    body.external_force_body
                        .iter()
                        .map(|v| v.e3)
                        .collect::<Vec<_>>(),
                );

                let external_torque_body_x = Series::new(
                    "external_torque_body_x",
                    body.external_torque_body
                        .iter()
                        .map(|v| v.e1)
                        .collect::<Vec<_>>(),
                );
                let external_torque_body_y = Series::new(
                    "external_torque_body_y",
                    body.external_torque_body
                        .iter()
                        .map(|v| v.e2)
                        .collect::<Vec<_>>(),
                );
                let external_torque_body_z = Series::new(
                    "external_torque_body_z",
                    body.external_torque_body
                        .iter()
                        .map(|v| v.e3)
                        .collect::<Vec<_>>(),
                );

                df.with_column(position_base_x).unwrap();
                df.with_column(position_base_y).unwrap();
                df.with_column(position_base_z).unwrap();
                df.with_column(velocity_base_x).unwrap();
                df.with_column(velocity_base_y).unwrap();
                df.with_column(velocity_base_z).unwrap();
                df.with_column(acceleration_base_x).unwrap();
                df.with_column(acceleration_base_y).unwrap();
                df.with_column(acceleration_base_z).unwrap();
                df.with_column(acceleration_body_x).unwrap();
                df.with_column(acceleration_body_y).unwrap();
                df.with_column(acceleration_body_z).unwrap();
                df.with_column(angular_accel_body_x).unwrap();
                df.with_column(angular_accel_body_y).unwrap();
                df.with_column(angular_accel_body_z).unwrap();
                df.with_column(angular_rate_body_x).unwrap();
                df.with_column(angular_rate_body_y).unwrap();
                df.with_column(angular_rate_body_z).unwrap();
                df.with_column(attitude_base_s).unwrap();
                df.with_column(attitude_base_x).unwrap();
                df.with_column(attitude_base_y).unwrap();
                df.with_column(attitude_base_z).unwrap();
                df.with_column(external_force_body_x).unwrap();
                df.with_column(external_force_body_y).unwrap();
                df.with_column(external_force_body_z).unwrap();
                df.with_column(external_torque_body_x).unwrap();
                df.with_column(external_torque_body_y).unwrap();
                df.with_column(external_torque_body_z).unwrap();
            }
            _ => panic!("Invalid component type"),
        }
        df
    }

    pub fn get_component_state(&self, component_name: &str, state_name: Vec<&str>) -> DataFrame {
        let df = self.get_component(component_name);
        let df_column_names = df.get_column_names();
        // check to make sure that the states exist in the dataframe
        // by getting intersection
        let state_names: Vec<&str> = {
            let df_set: HashSet<&str> = df_column_names.into_iter().collect();
            state_name
                .into_iter()
                .filter(|state| df_set.contains(state))
                .collect()
        };

        let mut columns: Vec<&str> = Vec::with_capacity(state_names.len() + 1);
        columns.push("t");
        columns.extend(state_names);

        df.select(columns).unwrap()
    }

    pub fn get_component_states(&self, component_name: &str) -> Vec<String> {
        let component = self.result.get(component_name).unwrap();
        match component {
            ResultEntry::Body(_) => {
                vec![
                    "position_base_x".to_string(),
                    "position_base_y".to_string(),
                    "position_base_z".to_string(),
                    "velocity_base_x".to_string(),
                    "velocity_base_y".to_string(),
                    "velocity_base_z".to_string(),
                    "acceleration_base_x".to_string(),
                    "acceleration_base_y".to_string(),
                    "acceleration_base_z".to_string(),
                    "acceleration_body_x".to_string(),
                    "acceleration_body_y".to_string(),
                    "acceleration_body_z".to_string(),
                    "angular_accel_body_x".to_string(),
                    "angular_accel_body_y".to_string(),
                    "angular_accel_body_z".to_string(),
                    "angular_rate_body_x".to_string(),
                    "angular_rate_body_y".to_string(),
                    "angular_rate_body_z".to_string(),
                    "attitude_base_s".to_string(),
                    "attitude_base_x".to_string(),
                    "attitude_base_y".to_string(),
                    "attitude_base_z".to_string(),
                    "external_force_body_x".to_string(),
                    "external_force_body_y".to_string(),
                    "external_force_body_z".to_string(),
                    "external_torque_body_x".to_string(),
                    "external_torque_body_y".to_string(),
                    "external_torque_body_z".to_string(),
                ]
            }
            ResultEntry::Joint(joint) => {
                match joint {
                    JointResult::Revolute(_) => {
                        vec!["theta".to_string(), "omega".to_string()]
                    }
                    JointResult::Prismatic(_) => {
                        vec!["position".to_string(), "velocity".to_string()]
                    } //_ => panic!("Invalid joint type"),
                }
            }
            ResultEntry::VecF64(_) => Vec::new(), //should not be possible
        }
    }

    pub fn get_components(&self) -> Vec<String> {
        self.result.keys().cloned().collect()
    }
}

#[derive(Debug)]
pub enum ResultEntry {
    Body(BodyResult),
    Joint(JointResult),
    VecF64(Vec<f64>),
}

impl fmt::Debug for MultibodyResult {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let time_start = DateTime::<Utc>::from(self.time_start);

        writeln!(f)?;
        writeln!(f, "sim name: {}", self.name)?;
        writeln!(f, "start time: {}", time_start)?;
        writeln!(f, "sim duration: {}", format_duration(self.sim_duration))?;
        writeln!(
            f,
            "total duration: {}",
            format_duration(self.total_duration)
        )?;
        writeln!(f, "states: ")?;

        // Collect headers (keys) and sort them
        let mut headers: Vec<&String> = self.result.keys().collect();
        headers.sort();

        // Print each header as an individual row
        for header in headers {
            writeln!(f, "     {}", header)?;
        }

        Ok(())
    }
}

pub fn update_body_states(bodies: &mut Vec<BodySim>, joints: &Vec<JointSim>) {
    for i in 0..bodies.len() {
        let body = &mut bodies[i];
        let inner_joint = &joints[i];
        let transforms = inner_joint.get_transforms();
        let body_from_joint = transforms.ob_from_jof;
        let base_from_body = transforms.base_from_jof * transforms.jof_from_ob;

        let joint_a = inner_joint.get_a();
        let body_a = body_from_joint * *joint_a;
        let body_a_in_base = base_from_body * body_a;

        let joint_v = inner_joint.get_v();
        let body_v = body_from_joint * *joint_v;
        let body_v_in_base = base_from_body * body_from_joint * *joint_v;

        body.state.acceleration_body = *body_a.translation();
        body.state.acceleration_base = *body_a_in_base.translation();
        body.state.angular_accel_body = *body_a.rotation();
        body.state.velocity_base = *body_v_in_base.translation();
        body.state.angular_rate_body = *body_v.rotation();

        let body_from_base = base_from_body.0.inv();
        body.state.position_base = body_from_base.translation.vec();
        body.state.attitude_base = Quaternion::from(body_from_base.rotation);
    }
}
