pub mod noise;

use ambassador::{delegatable_trait, Delegate};
use rate_gyro::RateGyro;
use star_tracker::StarTracker;
use crate::{
    body::{BodyConnection, BodyRef},
    MultibodyResult, ambassador_impl_MultibodyResult,
};
use csv::Writer;
use serde::{Deserialize, Serialize};
use utilities::initialize_writer;
use std::{fmt::Debug, fs::File, io::BufWriter, path::PathBuf};
use thiserror::Error;
use transforms::Transform;

pub mod rate_gyro;
pub mod star_tracker;

#[derive(Debug, Clone, Error)]
pub enum SensorErrors {
    #[error("sensor '{0}' is already connected to body '{1}'")]
    AlreadyConnectedToAnotherBody(String, String),
    #[error("sensor '{0}' is already connected to that body")]
    AlreadyConnectedToThisBody(String),
}


#[derive(Clone, Debug, Serialize, Delegate, Deserialize)]
#[delegate(SensorModelTrait)]
#[delegate(MultibodyResult)]
pub enum SensorModel {
    RateGyro(RateGyro),
    StarTracker(StarTracker),
}


#[delegatable_trait]
pub trait SensorModelTrait: MultibodyResult {
    fn update(&mut self, connection: &BodyConnection);
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sensor {
    pub name: String,
    pub model: SensorModel,
    connection: Option<BodyConnection>,
}

impl Sensor {
    pub fn connect_to_body(
        &mut self,
        body: &BodyRef,
        transform: Transform,
    ) -> Result<(), SensorErrors> {
        if let Some(connection) = &self.connection {
            return Err(SensorErrors::AlreadyConnectedToAnotherBody(
                self.name.clone(),
                connection.body.borrow().name.clone(),
            ));
        }

        self.connection = Some(BodyConnection::new(body.clone(), transform));
        Ok(())
    }

    pub fn new(name: &str, model: SensorModel) -> Self {
        Self {
            name: name.to_string(),
            model,
            connection: None,
        }
    }

    pub fn update(&mut self) {
        self.model.update(&self.connection.as_ref().unwrap());
    }

    pub fn initialize_writer(&self, result_folder_path: &PathBuf) -> Writer<BufWriter<File>> {
        // Define the sensor subfolder folder path
        let sensor_folder_path = result_folder_path.join("sensors");

        // Check if the folder exists, if not, create it
        if !sensor_folder_path.exists() {
            std::fs::create_dir_all(&sensor_folder_path).expect("Failed to create sensor folder");
        }

        // Initialize writer using the updated path
        initialize_writer(self.name.clone(), &sensor_folder_path)
    }
}
