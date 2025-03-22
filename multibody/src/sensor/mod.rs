pub mod noise;

use crate::{
    body::{Body, BodyConnection},
    system::Id,
};
use gps::Gps;
use indexmap::IndexMap;
use magnetometer::Magnetometer;
use nadir_result::{NadirResult, ResultManager};
use rate_gyro::RateGyro;
use serde::{Deserialize, Serialize};
use star_tracker::StarTracker;
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
    fn update(&mut self, body: &Body, body_transform: &Transform);
    fn result_headers(&self) -> &[&str];
    fn result_content(&self, id: u32, results: &mut ResultManager);
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sensor {
    pub name: String,
    pub model: SensorModels,
    pub connection: Option<BodyConnection>,
    result_id: Option<u32>,
}

impl Sensor {
    pub fn connect_to_body(&mut self, body: Id, transform: Transform) -> Result<(), SensorErrors> {
        if let Some(connection) = &self.connection {
            return Err(SensorErrors::AlreadyConnectedToAnotherBody(
                self.name.clone(),
                connection.body.to_string(),
            ));
        }

        self.connection = Some(BodyConnection::new(body.clone(), transform));
        Ok(())
    }

    pub fn new(name: &str, model: SensorModels) -> Self {
        Self {
            name: name.to_string(),
            model,
            connection: None,
            result_id: None,
        }
    }

    pub fn update(&mut self, bodies: &IndexMap<Id, Body>) {
        if let Some(connection) = &self.connection {
            if let Some(body) = bodies.get(&connection.body) {
                self.model.update(body, &connection.transform);
            }
        }
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

    fn update(&mut self, body: &Body, body_transform: &Transform) {
        match self {
            SensorModels::Gps(sensor) => sensor.update(body, body_transform),
            SensorModels::Magnetometer(sensor) => sensor.update(body, body_transform),
            SensorModels::RateGyro(sensor) => sensor.update(body, body_transform),
            SensorModels::StarTracker(sensor) => sensor.update(body, body_transform),
        }
    }
}
