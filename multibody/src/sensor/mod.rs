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

#[typetag::serde]
pub trait SensorModel: CloneSensorModel + Debug + MultibodyResult {
    fn update(&mut self, connection: &BodyConnection);
}
pub trait CloneSensorModel {
    fn clone_model(&self) -> Box<dyn SensorModel>;
}
impl<T> CloneSensorModel for T
where
    T: SensorModel + Clone + 'static,
{
    fn clone_model(&self) -> Box<dyn SensorModel> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn SensorModel> {
    fn clone(&self) -> Self {
        self.clone_model()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sensor {
    pub name: String,
    pub model: Box<dyn SensorModel>,
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

    pub fn new(name: &str, model: impl SensorModel + 'static) -> Self {
        Self {
            name: name.to_string(),
            model: Box::new(model),
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
