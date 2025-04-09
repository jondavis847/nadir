use aerospace::orbit::KeplerianElements;
use celestial::CelestialBodies;
use nadir_result::{NadirResult, ResultManager};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use time::{Time, TimeSystem};

use crate::software::sensors::gps::GpsFsw;

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct OrbitDetermination {
    pub state: State,
    parameters: Parameters,
    result_id: Option<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct State {
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    orbit: KeplerianElements,
    pub sun_position: Vector3<f64>,
}

impl Default for State {
    fn default() -> Self {
        let orbit = KeplerianElements::new(
            1e8,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            Time::from_sec_j2k(0.0, time::TimeSystem::TAI),
            CelestialBodies::Earth,
        );
        let (position, velocity) = orbit.get_rv();

        //TODO: impl vallado luni solar for fsw
        let sun_position = Vector3::zeros();
        Self {
            position,
            velocity,
            orbit,
            sun_position,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
struct Parameters {
    //propagator:
}

impl OrbitDetermination {
    pub fn run(&mut self, gps: &GpsFsw) {
        if gps.state.valid {
            self.state.position = gps.state.position;
            self.state.velocity = gps.state.velocity;
            self.state.orbit = KeplerianElements::from_rv(
                gps.state.position,
                gps.state.velocity,
                gps.state.time.to_system(TimeSystem::TAI),
                CelestialBodies::Earth,
            );
        } else {
            unimplemented!("need to implement orbit propagator")
        }
    }
}

impl NadirResult for OrbitDetermination {
    fn new_result(&mut self, results: &mut ResultManager) {
        // Define the actuator subfolder folder path
        let fsw_folder_path = results.result_path.join("software");

        // Check if the folder exists, if not, create it
        if !fsw_folder_path.exists() {
            std::fs::create_dir_all(&fsw_folder_path).expect("Failed to create fsw folder");
        }

        // Initialize writer using the updated path
        let headers = [
            "position[x]",
            "position[y]",
            "position[z]",
            "velocity[x]",
            "velocity[y]",
            "velocity[z]",
        ];
        let id = results.new_writer("fsw_nav_od", &fsw_folder_path, &headers);
        self.result_id = Some(id);
    }

    fn write_result(&self, results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            results.write_record(
                id,
                &[
                    self.state.position[0].to_string(),
                    self.state.position[1].to_string(),
                    self.state.position[2].to_string(),
                    self.state.velocity[0].to_string(),
                    self.state.velocity[1].to_string(),
                    self.state.velocity[2].to_string(),
                ],
            );
        }
    }
}
