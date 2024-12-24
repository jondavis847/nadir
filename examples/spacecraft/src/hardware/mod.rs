use std::{fs::File, io::BufWriter, path::PathBuf};

use csv::Writer;
use multibody::{sensor::{rate_gyro::RateGyro, star_tracker::StarTracker, Sensor, SensorSystem}, MultibodyResult};
use serde::{Deserialize, Serialize};

#[derive(Debug,Clone,Deserialize,Serialize)]
pub struct SpacecraftSensors{
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