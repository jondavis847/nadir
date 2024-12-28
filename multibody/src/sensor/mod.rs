pub mod noise;

use crate::body::{BodyConnection, BodyRef};
use nadir_result::{NadirResult, ResultManager};
use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use thiserror::Error;
use transforms::Transform;

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
pub struct Sensor<T> where T: SensorModel {
    pub name: String,
    pub model: T,
    connection: Option<BodyConnection>,
    result_id: Option<u32>,
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
            result_id: None,
        }
    }

    pub fn update(&mut self) {
        self.model.update(&self.connection.as_ref().unwrap());
    }
}

impl<T> NadirResult for Sensor<T> where T: SensorModel {
    fn new_result(&mut self,results: &mut ResultManager) {
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

    fn write_result(&self,results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            self.model.result_content(id,results);            
        }    
    }
}

pub trait SensorSystem: Serialize {
    fn update(&mut self);
    fn initialize_results(&mut self, results: &mut ResultManager);
    fn write_results(&self, results: &mut ResultManager);
}

impl SensorSystem for () {
    fn update(&mut self) {}
    fn initialize_results(&mut self, _results: &mut ResultManager) {}
    fn write_results(&self, _results: &mut ResultManager) {}
}