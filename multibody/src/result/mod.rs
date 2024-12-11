use crate::base::BaseErrors;
use crate::system::MultibodySystem;
use crate::MultibodyErrors;
use aerospace::celestial_system::{CelestialBodies, CelestialErrors, CelestialResult};
use bincode::{serialize, serialize_into};
use chrono::{DateTime, Utc};
use nadir_3d::mesh::Mesh;
use nalgebra::Vector3;
use ron::ser::{to_string_pretty, PrettyConfig};
use rotations::quaternion::Quaternion;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fmt;
use std::fs::File;
use std::io::Write;
use std::time::{Duration, SystemTime};
use utilities::format_duration;

pub trait MultibodyResultTrait {
    /// Initializes the ResultEntry for storing the sim result for this joint
    fn initialize_result(&mut self, capacity: usize);
    /// Collects the final joint model result to be added to the MultibodyResult map
    fn get_result_entry(&mut self) -> ResultEntry;
    // Updates the result entry with the values from the joint
    fn update_result(&mut self);
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct ResultEntry(HashMap<String, Vec<f64>>);

impl ResultEntry {
    pub fn new(map: HashMap<String, Vec<f64>>) -> Self {
        Self(map)
    }
    pub fn get(&self, state: &str) -> Result<&Vec<f64>, MultibodyErrors> {
        self.0
            .get(state)
            .ok_or(MultibodyErrors::ComponentStateNotFound(state.to_string()))
    }

    pub fn keys(&self) -> Vec<String> {
        self.0.keys().cloned().collect()
    }
}

#[derive(Clone, Serialize, Deserialize)]
pub struct MultibodyResult {
    pub name: String,
    pub time_start: SystemTime,
    pub sim_duration: Duration,
    pub total_duration: Duration,
    pub sim_time: Vec<f64>,
    pub result: HashMap<String, ResultEntry>,
    pub bodies: HashMap<String, Mesh>, // need to keep the bodies for animation initialization and mesh info
    pub celestial: Option<CelestialResult>, // need to know which celestial bodies to render
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
        if let Some(result) = self.get_component(component_name) {
            if let Some(entry) = result.0.get(state_name) {
                Ok(entry)
            } else {
                Err(MultibodyErrors::ComponentStateNotFound(
                    state_name.to_string(),
                ))
            }
        } else {
            Err(MultibodyErrors::ComponentNotFound(
                component_name.to_string(),
            ))
        }
    }

    pub fn get_component_states(
        &self,
        component_name: &str,
    ) -> Result<Vec<String>, MultibodyErrors> {
        if let Some(component) = self.result.get(component_name) {
            Ok(component.0.keys().cloned().collect())
        } else {
            Err(MultibodyErrors::ComponentNotFound(
                component_name.to_string(),
            ))
        }
    }

    pub fn get_body_state_at_time_interp(
        &self,
        body_name: &str,
        t: f64,
    ) -> Result<(Quaternion, Vector3<f64>), MultibodyErrors> {
        if let Some(result) = self.result.get(body_name) {
            let time = &self.sim_time;
            let qx = result.get("attitude[x]{base}")?;
            let qy = result.get("attitude[y]{base}")?;
            let qz = result.get("attitude[z]{base}")?;
            let qw = result.get("attitude[w]{base}")?;

            let rx = result.get("position[x]{base}")?;
            let ry = result.get("position[y]{base}")?;
            let rz = result.get("position[z]{base}")?;

            let attitude = (qx, qy, qz, qw);
            let position = (rx, ry, rz);

            Ok(get_state_at_time_interp(t, time, &attitude, &position))
        } else {
            Err(MultibodyErrors::ComponentNotFound(body_name.to_string()))
        }
    }

    pub fn get_celestial_state_at_time_interp(
        &self,
        body: CelestialBodies,
        t: f64,
    ) -> Result<(Quaternion, Vector3<f64>), MultibodyErrors> {
        let time = &self.sim_time;

        if let Some(celestial) = &self.celestial {
            if let Some(body) = celestial.bodies.get(&body) {
                let qx = body.q.iter().map(|q| q.x).collect();
                let qy = body.q.iter().map(|q| q.y).collect();
                let qz = body.q.iter().map(|q| q.z).collect();
                let qw = body.q.iter().map(|q| q.s).collect();
                let rx = body.r.iter().map(|r| r[0]).collect();
                let ry = body.r.iter().map(|r| r[1]).collect();
                let rz = body.r.iter().map(|r| r[2]).collect();

                let attitude = (&qx, &qy, &qz, &qw);
                let position = (&rx, &ry, &rz);
                Ok(get_state_at_time_interp(t, time, &attitude, &position))
            } else {
                Err(MultibodyErrors::CelestialErrors(
                    CelestialErrors::BodyNotFoundInCelestialSystem,
                ))
            }
        } else {
            Err(MultibodyErrors::BaseErrors(BaseErrors::BaseIsNotCelestial))
        }
    }

    pub fn get_components(&self) -> Vec<String> {
        self.result.keys().cloned().collect()
    }

    pub fn save(&self, sys: &MultibodySystem) {
        // check if there is a results folder, otherwise make one
        let mut path = std::path::PathBuf::new();
        path.push("results");
        if !path.exists() {
            std::fs::create_dir_all(&path).unwrap();
        }

        // check if this folder exists, panic if so TODO: Put this before starting the sim!
        path.push(&self.name);
        if !path.exists() {
            std::fs::create_dir_all(&path).unwrap();
        }

        // // Serialize `sys` to a binary file
        // let sys_name = self.name.clone() + ".sys";
        // let sys_path = path.join(sys_name);
        // let mut file = File::create(sys_path).unwrap();
        // serialize_into(&mut file, &sys).unwrap();

        // Serialize another struct or result
        let res_path = path.join("result.bin");
        let mut file = File::create(res_path).unwrap();
        serialize_into(&mut file, &self).unwrap(); // Replace `sys` with the actual struct

        // create the sys file

        let sys_path = path.join("system.ron");
        let mut file = File::create(sys_path).unwrap();
        let ron_string = to_string_pretty(sys, PrettyConfig::new()).unwrap();
        file.write_all(ron_string.as_bytes()).unwrap();

        // // create the ron file
        // let res_path = path.join("result.ron");
        // let mut file = File::create(res_path).unwrap();
        // let ron_string = to_string_pretty(self, PrettyConfig::new()).unwrap();
        // file.write_all(ron_string.as_bytes()).unwrap();
    }
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
