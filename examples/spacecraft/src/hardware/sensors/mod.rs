
pub mod gps;
pub mod rate_gyro;
pub mod star_tracker;

use std::{fs::File, io::BufWriter, path::PathBuf};

use csv::Writer;
use gps::Gps;
use multibody::{sensor::{Sensor, SensorSystem}, MultibodyResult};
use rate_gyro::RateGyro;
use serde::{Deserialize, Serialize};
use star_tracker::StarTracker;

#[derive(Debug,Clone,Deserialize,Serialize)]
pub struct SpacecraftSensors{
    pub gps: Sensor<Gps>,
    pub st: Sensor<StarTracker>,
    pub imu: Sensor<RateGyro>,
}

impl SensorSystem for SpacecraftSensors {
    fn update(&mut self) {
        self.st.update();
        self.imu.update();
    }

    fn initialize_writers(&self, path: &PathBuf) -> Vec<Writer<BufWriter<File>>> {
        let mut writers = Vec::new();
        writers.push(self.imu.initialize_writer(path));
        writers.push(self.st.initialize_writer(path));
        writers
    }

    fn write_result_files(&self, writers: &mut Vec<Writer<BufWriter<File>>>) {
        // make sure order matches the order you pushed the writers to in initialize_writers
        self.imu.model.write_result_file(&mut writers[0]);
        self.st.model.write_result_file(&mut writers[1]);
    }
}