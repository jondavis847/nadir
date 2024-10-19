use aerospace::celestial_system::CelestialResult;
use chrono::{DateTime, Utc};
use nalgebra::Vector3;
use rotations::quaternion::Quaternion;
use serde::{Deserialize, Serialize};
use std::collections::{HashMap, HashSet};
use std::fmt;
use std::time::{Duration, SystemTime};
use utilities::format_duration;

use polars::prelude::*;

use crate::system_sim::MultibodySystemSim;
use crate::{body::BodyResult, joint::JointResult, sensor::SensorResult};

pub trait MultibodyResultTrait {
    fn get_state_names(&self) -> Vec<&'static str>;
    fn get_result_entry(&self) -> ResultEntry;
    fn add_to_dataframe(&self, df: &mut DataFrame);
}

#[derive(Clone, Serialize, Deserialize)]
pub struct MultibodyResult {
    pub name: String,
    pub system: MultibodySystemSim,
    pub sim_time: Vec<f64>,
    pub result: HashMap<String, ResultEntry>,
    pub time_start: SystemTime,
    pub sim_duration: Duration,
    pub total_duration: Duration,
}

impl MultibodyResult {
    pub fn get_component(&self, component_name: &str) -> Option<DataFrame> {
        if let Some(component) = self.result.get(component_name) {
            let mut df = DataFrame::default();

            let t = Series::new("t", self.sim_time.clone());
            df.with_column(t).unwrap();

            match component {
                ResultEntry::Joint(joint) => joint.add_to_dataframe(&mut df),
                ResultEntry::Celestial(celestial) => celestial.add_to_dataframe(&mut df),
                ResultEntry::Body(body) => body.add_to_dataframe(&mut df),
                ResultEntry::Sensor(sensor) => sensor.add_to_dataframe(&mut df),       
                ResultEntry::VecF64(_) => todo!("this is only for time vector, which was alreay added to df. this should probably just be a result type"),
            }
            Some(df)
        } else {
            None
        }
    }

    pub fn get_component_state(&self, component_name: &str, state_name: Vec<&str>) -> Option<DataFrame> {
        if let Some(df) = self.get_component(component_name) {
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

            df.select(columns).unwrap();
            Some(df)
        } else {
            None
        }
    }

    pub fn get_component_states(&self, component_name: &str) -> Vec<&'static str> {
        let component = self.result.get(component_name).unwrap();
        match component {
            ResultEntry::Body(result) => result.get_state_names(),
            ResultEntry::Joint(result) => result.get_state_names(),
            ResultEntry::Sensor(result) => result.get_state_names(),
            ResultEntry::VecF64(_) => Vec::new(), //should not be possible
            ResultEntry::Celestial(result) => result.get_state_names(),
        }
    }

    pub fn get_body_state_at_time_interp(
        &self,
        body_name: &str,
        t: f64,
    ) -> (Quaternion, Vector3<f64>) {
        let body = match self.result.get(body_name).unwrap() {
            ResultEntry::Body(body) => body,
            _ => panic!("should not be possible"),
        };

        let position = &body.position_base;
        let attitude = &body.attitude_base;

        let time = &self.sim_time;
        match time.binary_search_by(|v| v.partial_cmp(&t).unwrap()) {
            Ok(i) => {
                // The target is exactly at index i
                (attitude[i], position[i])
            }
            Err(i) => {
                if i == 0 {
                    // The target is smaller than the first element
                    (attitude[i], position[i])
                } else if i == time.len() {
                    // The target is greater than the last element
                    (attitude[i], position[i])
                } else {
                    // The target is between elements at i - 1 and i
                    let t_prev = time[i - 1];
                    let t_next = time[i];
                    let interp_factor = (t - t_prev) / (t_next - t_prev); // between 0-1

                    let position_prev = position[i - 1];
                    let position_next = position[i];
                    let interp_position =
                        Vector3::lerp(&position_prev, &position_next, interp_factor);

                    let attitude_prev = attitude[i - 1];
                    let mut attitude_next = attitude[i];

                    // Ensure quaternion continuity
                    if attitude_prev.dot(&attitude_next) < 0.0 {
                        attitude_next = Quaternion {
                            x: -attitude_next.x,
                            y: -attitude_next.y,
                            z: -attitude_next.z,
                            s: -attitude_next.s,
                        };
                    }

                    let interp_attitude;
                    if false {
                        // TODO: fix squad
                        //if i >= 2 && i < attitude.len() - 1 {
                        //squad
                        let q0 = attitude[i - 2];
                        let q3 = attitude[i + 1];
                        interp_attitude =
                            Quaternion::squad(q0, attitude_prev, attitude_next, q3, interp_factor);
                    } else {
                        //slerp
                        interp_attitude =
                            Quaternion::slerp(&attitude_prev, &attitude_next, interp_factor);
                    }

                    (interp_attitude, interp_position)
                }
            }
        }
    }
    pub fn get_components(&self) -> Vec<String> {
        self.result.keys().cloned().collect()
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ResultEntry {
    Body(BodyResult),
    Celestial(CelestialResult),
    Joint(JointResult),
    VecF64(Vec<f64>),
    Sensor(SensorResult),
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
