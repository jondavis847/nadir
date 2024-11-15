use crate::system_sim::MultibodySystemSim;
use crate::MultibodyErrors;
use crate::{body::BodyResult, joint::JointResult, sensor::SensorResult};
use aerospace::celestial_system::{CelestialBodies, CelestialErrors, CelestialResult};
use chrono::{DateTime, Utc};
use nalgebra::Vector3;
use rotations::quaternion::Quaternion;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;
use std::time::{Duration, SystemTime};
use utilities::format_duration;

pub trait MultibodyResultTrait {
    const STATES: &'static [&'static str];
    type Component;
    fn get_state_names(&self) -> &'static [&'static str];
    fn get_state_value(&self, state: &str) -> Result<&Vec<f64>, MultibodyErrors>;
    fn get_result_entry(&self) -> ResultEntry;
    fn update(&mut self, component: &Self::Component);
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
    pub fn get_component(&self, component_name: &str) -> Option<&ResultEntry> {
        self.result.get(component_name)
    }

    pub fn get_component_state(
        &self,
        component_name: &str,
        state_name: &str,
    ) -> Result<&Vec<f64>, MultibodyErrors> {
        if let Some(entry) = self.get_component(component_name) {
            match entry {
                ResultEntry::Body(result) => result.get_state_value(state_name),
                ResultEntry::Joint(result) => result.get_state_value(state_name),
                ResultEntry::Sensor(result) => result.get_state_value(state_name),
                ResultEntry::Celestial(result) => Ok(result.get_state_value(state_name)?),
            }
        } else {
            Err(MultibodyErrors::ComponentNotFound(
                component_name.to_string(),
            ))
        }
    }

    pub fn get_component_states(&self, component_name: &str) -> Vec<String> {
        let component = self.result.get(component_name).unwrap();
        match component {
            ResultEntry::Body(result) => result
                .get_state_names()
                .iter()
                .map(|state| state.to_string())
                .collect(),
            ResultEntry::Joint(result) => result
                .get_state_names()
                .iter()
                .map(|state| state.to_string())
                .collect(),
            ResultEntry::Sensor(result) => result
                .get_state_names()
                .iter()
                .map(|state| state.to_string())
                .collect(),
            ResultEntry::Celestial(result) => result.get_state_names(),
        }
    }

    pub fn get_body_state_at_time_interp(
        &self,
        body_name: &str,
        t: f64,
    ) -> Result<(Quaternion, Vector3<f64>), MultibodyErrors> {
        let body = match self.result.get(body_name).unwrap() {
            ResultEntry::Body(body) => body,
            _ => unreachable!("should not be possible"),
        };
        let time = &self.sim_time;
        let qx = body.get_state_value("attitude[x]{base}")?;
        let qy = body.get_state_value("attitude[y]{base}")?;
        let qz = body.get_state_value("attitude[z]{base}")?;
        let qw = body.get_state_value("attitude[w]{base}")?;
        let rx = body.get_state_value("position[x]{base}")?;
        let ry = body.get_state_value("position[y]{base}")?;
        let rz = body.get_state_value("position[z]{base}")?;

        let attitude = (qx, qy, qz, qw);
        let position = (rx, ry, rz);

        Ok(get_state_at_time_interp(t, time, &attitude, &position))
    }

    pub fn get_celestial_state_at_time_interp(
        &self,
        body: CelestialBodies,
        t: f64,
    ) -> Result<(Quaternion, Vector3<f64>), MultibodyErrors> {
        let time = &self.sim_time;
        let celestial = match self.result.get("celestial").unwrap() {
            ResultEntry::Celestial(celestial) => celestial,
            _ => unreachable!("should not be possible"),
        };

        if let Some(body_result) = celestial.bodies.get(&body) {
            let qx = body_result.get_state_value("orientation[x]")?;
            let qy = body_result.get_state_value("orientation[y]")?;
            let qz = body_result.get_state_value("orientation[z]")?;
            let qw = body_result.get_state_value("orientation[w]")?;
            let rx = body_result.get_state_value("position[x]")?;
            let ry = body_result.get_state_value("position[y]")?;
            let rz = body_result.get_state_value("position[z]")?;

            let attitude = (qx, qy, qz, qw);
            let position = (rx, ry, rz);

            Ok(get_state_at_time_interp(t, time, &attitude, &position))
        } else {
            Err(MultibodyErrors::CelestialErrors(
                CelestialErrors::BodyNotFoundInCelestialSystem,
            ))
        }
    }

    pub fn get_components(&self) -> Vec<String> {
        self.result.keys().cloned().collect()
    }
}

fn get_state_at_time_interp(
    t: f64,
    time: &Vec<f64>,
    q: &(&Vec<f64>, &Vec<f64>, &Vec<f64>, &Vec<f64>),
    r: &(&Vec<f64>, &Vec<f64>, &Vec<f64>),
) -> (Quaternion, Vector3<f64>) {
    match time.binary_search_by(|v| v.partial_cmp(&t).unwrap()) {
        Ok(i) => {
            // The target is exactly at index i
            (
                Quaternion::new(q.0[i], q.1[i], q.2[i], q.3[i]),
                Vector3::new(r.0[i], r.1[i], r.2[i]),
            )
        }
        Err(i) => {
            if i == 0 {
                // The target is smaller than the first element
                (
                    Quaternion::new(q.0[i], q.1[i], q.2[i], q.3[i]),
                    Vector3::new(r.0[i], r.1[i], r.2[i]),
                )
            } else if i == time.len() {
                // The target is greater than the last element
                (
                    Quaternion::new(q.0[i], q.1[i], q.2[i], q.3[i]),
                    Vector3::new(r.0[i], r.1[i], r.2[i]),
                )
            } else {
                // The target is between elements at i - 1 and i
                let t_prev = time[i - 1];
                let t_next = time[i];
                let interp_factor = (t - t_prev) / (t_next - t_prev); // between 0-1

                let position_prev = Vector3::new(r.0[i - 1], r.1[i - 1], r.2[i - 1]);
                let position_next = Vector3::new(r.0[i], r.1[i], r.2[i]);
                let interp_position = Vector3::lerp(&position_prev, &position_next, interp_factor);

                let attitude_prev = Quaternion::new(q.0[i - 1], q.1[i - 1], q.2[i - 1], q.3[i - 1]);
                let mut attitude_next = Quaternion::new(q.0[i], q.1[i], q.2[i], q.3[i]);

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
                    let q0 = Quaternion::new(q.0[i - 2], q.1[i - 2], q.2[i - 2], q.3[i - 2]);
                    let q3 = Quaternion::new(q.0[i + 1], q.1[i + 1], q.2[i + 1], q.3[i + 1]);
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

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum ResultEntry {
    Body(BodyResult),
    Celestial(CelestialResult),
    Joint(JointResult),
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
