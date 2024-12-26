use csv::Writer;
use serde::{Deserialize, Serialize};
use spatial_algebra::Force;
use thiserror::Error;
use transforms::Transform;
use utilities::initialize_writer;
use std::{fmt::Debug, fs::File, io::BufWriter, path::PathBuf};

use crate::{body::{BodyConnection, BodyRef}, MultibodyResult};

#[derive(Debug, Clone, Error)]
pub enum ActuatorErrors {
    #[error("actuator '{0}' is already connected to body '{1}'")]
    AlreadyConnectedToAnotherBody(String, String),
    #[error("actuator '{0}' is already connected to that body")]
    AlreadyConnectedToThisBody(String),
}

pub trait ActuatorModel: MultibodyResult {
    type Command;
    fn process_command(&mut self, command: &Self::Command, connection: &BodyConnection);
    fn get_force_body(&self) -> Force;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Actuator<T> where T: ActuatorModel {
    pub name: String,
    pub model: T,
    pub connection: Option<BodyConnection>,
}

impl<T> Actuator<T> where T: ActuatorModel {
    pub fn new(name: &str, model: T) -> Self {
        Self {
            name: name.to_string(),
            model,
            connection: None,
        }
    }
    pub fn connect_to_body(
        &mut self,
        body: &BodyRef,
        transform: Transform,
    ) -> Result<(), ActuatorErrors> {
        if let Some(connection) = &self.connection {
            return Err(ActuatorErrors::AlreadyConnectedToAnotherBody(
                self.name.clone(),
                connection.body.borrow().name.clone(),
            ));
        }

        self.connection = Some(BodyConnection::new(body.clone(), transform));
        Ok(())
    }

    pub fn initialize_writer(&self, result_folder_path: &PathBuf) -> Writer<BufWriter<File>> {
        // Define the actuator subfolder folder path
        let actuator_folder_path = result_folder_path.join("actuators");

        // Check if the folder exists, if not, create it
        if !actuator_folder_path.exists() {
            std::fs::create_dir_all(&actuator_folder_path).expect("Failed to create actuator folder");
        }

        // Initialize writer using the updated path
        initialize_writer(self.name.clone(), &actuator_folder_path)
    }
}

pub trait ActuatorSystem: Serialize {
    fn update(&mut self);
    fn initialize_writers(&self, path: &PathBuf) -> Vec<Writer<BufWriter<File>>>;
    fn write_result_files(&self, writers: &mut Vec<Writer<BufWriter<File>>>);
}