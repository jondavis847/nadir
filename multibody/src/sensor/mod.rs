pub mod noise;

use crate::{
    body::{BodyConnection, BodyRef},
    MultibodyResult,
};
use csv::Writer;
use serde::{Deserialize, Serialize};
use utilities::initialize_writer;
use std::{fmt::Debug, fs::File, io::BufWriter, path::PathBuf};
use thiserror::Error;
use transforms::Transform;

#[derive(Debug, Clone, Error)]
pub enum SensorErrors {
    #[error("sensor '{0}' is already connected to body '{1}'")]
    AlreadyConnectedToAnotherBody(String, String),
    #[error("sensor '{0}' is already connected to that body")]
    AlreadyConnectedToThisBody(String),
}

pub trait SensorModel: MultibodyResult {
    fn update(&mut self, connection: &BodyConnection);
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sensor<T> where T: SensorModel {
    pub name: String,
    pub model: T,
    connection: Option<BodyConnection>,
}

impl<T> Sensor<T> where T: SensorModel {
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

    pub fn new(name: &str, model: T) -> Self where T: SensorModel {
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


pub trait SensorSystem: Serialize {
    fn update(&mut self);
    fn initialize_writers(&self, path: &PathBuf) -> Vec<Writer<BufWriter<File>>>;
    fn write_result_files(&self, writers: &mut Vec<Writer<BufWriter<File>>>);
}