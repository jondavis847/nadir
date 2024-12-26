use gps::GpsFsw;
use imu::RateGyroFsw;
use serde::{Deserialize, Serialize};
use star_tracker::StarTrackerFsw;

use crate::hardware::sensors::SpacecraftSensors;

mod gps;
mod imu;
mod star_tracker;

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
}