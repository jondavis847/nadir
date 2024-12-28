
pub mod gps;
pub mod rate_gyro;
pub mod star_tracker;

use gps::Gps;
use multibody::sensor::{Sensor, SensorSystem};
use rate_gyro::RateGyro;
use serde::{Deserialize, Serialize};
use star_tracker::StarTracker;
use nadir_result::NadirResult;

#[derive(Debug,Clone,Deserialize,Serialize)]
pub struct SpacecraftSensors{
    pub gps: Sensor<Gps>,
    pub st: Sensor<StarTracker>,
    pub imu: Sensor<RateGyro>,
}

impl SensorSystem for SpacecraftSensors {
    fn update(&mut self) {
        self.gps.update();
        self.st.update();
        self.imu.update();
    }

    fn initialize_results(&mut self, results: &mut nadir_result::ResultManager) {
        self.gps.new_result(results);
        self.st.new_result(results);
        self.imu.new_result(results);
    }

    fn write_results(&self, results: &mut nadir_result::ResultManager) {
        self.gps.write_result(results);
        self.st.write_result(results);
        self.imu.write_result(results);
    }
}