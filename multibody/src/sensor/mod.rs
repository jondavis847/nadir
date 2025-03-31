pub mod noise;

use crate::{
    body::{BodyConnection, BodyConnectionBuilder},
    system::Id,
};
use gps::{Gps, GpsBuilder};
use magnetometer::{Magnetometer, MagnetometerBuilder};
use nadir_result::{NadirResult, ResultManager};
use rate_gyro::{RateGyro, RateGyroBuilder};
use serde::{Deserialize, Serialize};
use star_tracker::{StarTracker, StarTrackerBuilder};
use std::fmt::Debug;
use thiserror::Error;
use transforms::Transform;

pub mod gps;
pub mod magnetometer;
pub mod rate_gyro;
pub mod star_tracker;

#[derive(Debug, Clone, Error)]
pub enum SensorErrors {
    #[error("sensor '{0}' is already connected to body '{1}'")]
    AlreadyConnectedToAnotherBody(String, String),
    #[error("sensor '{0}' is already connected to that body")]
    AlreadyConnectedToThisBody(String),
}

pub trait SensorModel {
    fn update(&mut self, connection: &BodyConnection);
    fn result_headers(&self) -> &[&str];
    fn result_content(&self, id: u32, results: &mut ResultManager);
}
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct SensorBuilder {
    pub name: String,
    pub model: SensorModelBuilders,
    pub connection: Option<BodyConnectionBuilder>,
}

impl SensorBuilder {
    pub fn connect_body(&mut self, body: Id, transform: Transform) -> Result<(), SensorErrors> {
        self.connection = Some(BodyConnectionBuilder::new(body.clone(), transform));
        Ok(())
    }

    pub fn new(name: &str, model: SensorModelBuilders) -> Self {
        Self {
            name: name.to_string(),
            model,
            connection: None,
        }
    }
}

#[derive(Debug)]
pub struct Sensor {
    pub name: String,
    pub model: SensorModels,
    pub connection: BodyConnection,
    result_id: Option<u32>,
}

impl Sensor {
    pub fn update(&mut self) {
        self.model.update(&self.connection);
    }
}

impl NadirResult for Sensor {
    fn new_result(&mut self, results: &mut ResultManager) {
        // Define the sensor subfolder folder path
        let sensor_folder_path = results.result_path.join("sensors");

        // Check if the folder exists, if not, create it
        if !sensor_folder_path.exists() {
            std::fs::create_dir_all(&sensor_folder_path).expect("Failed to create sensor folder");
        }

        // Initialize writer using the updated path
        let id = results.new_writer(&self.name, &sensor_folder_path, self.model.result_headers());
        self.result_id = Some(id);
    }

    fn write_result(&self, results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            self.model.result_content(id, results);
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SensorModelBuilders {
    Gps(GpsBuilder),
    Magnetometer(MagnetometerBuilder),
    RateGyro(RateGyroBuilder),
    StarTracker(StarTrackerBuilder),
}

#[derive(Debug)]
pub enum SensorModels {
    Gps(Gps),
    Magnetometer(Magnetometer),
    RateGyro(RateGyro),
    StarTracker(StarTracker),
}

impl SensorModel for SensorModels {
    fn result_content(&self, id: u32, results: &mut ResultManager) {
        match self {
            SensorModels::Gps(sensor) => sensor.result_content(id, results),
            SensorModels::Magnetometer(sensor) => sensor.result_content(id, results),
            SensorModels::RateGyro(sensor) => sensor.result_content(id, results),
            SensorModels::StarTracker(sensor) => sensor.result_content(id, results),
        }
    }

    fn result_headers(&self) -> &[&str] {
        match self {
            SensorModels::Gps(sensor) => sensor.result_headers(),
            SensorModels::Magnetometer(sensor) => sensor.result_headers(),
            SensorModels::RateGyro(sensor) => sensor.result_headers(),
            SensorModels::StarTracker(sensor) => sensor.result_headers(),
        }
    }

    fn update(&mut self, connection: &BodyConnection) {
        match self {
            SensorModels::Gps(sensor) => sensor.update(connection),
            SensorModels::Magnetometer(sensor) => sensor.update(connection),
            SensorModels::RateGyro(sensor) => sensor.update(connection),
            SensorModels::StarTracker(sensor) => sensor.update(connection),
        }
    }
}
