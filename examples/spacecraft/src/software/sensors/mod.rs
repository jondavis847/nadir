use gps::GpsFsw;
use imu::RateGyroFsw;
use serde::{Deserialize, Serialize};
use star_tracker::StarTrackerFsw;
use nadir_result::{NadirResult, ResultManager};

use crate::hardware::sensors::SpacecraftSensors;

pub mod gps;
pub mod imu;
pub mod star_tracker;

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct SensorFsw {
    pub gps: GpsFsw,
    pub imu: RateGyroFsw,
    pub st: StarTrackerFsw    
}

impl SensorFsw {
    pub fn run(&mut self, sensors: &SpacecraftSensors) {        
        self.gps.run(&sensors.gps.model);
        self.imu.run(&sensors.imu.model);
        self.st.run(&sensors.st.model);
    }

    pub fn write_results(&self, results: &mut ResultManager) {
        self.gps.write_result(results);
        self.imu.write_result(results);
        self.st.write_result(results);
    }

    pub fn initialize_results(&mut self, results: &mut ResultManager) {
        self.gps.new_result(results);
        self.imu.new_result(results);
        self.st.new_result(results);
    }
}