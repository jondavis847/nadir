use nadir_result::{NadirResult, ResultManager};
use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use thiserror::Error;
use transforms::Transform;

use crate::{
    body::{BodyConnection, BodyRef},
    solver::{SimStateVector, SimStates},
};

#[derive(Debug, Clone, Error)]
pub enum ActuatorErrors {
    #[error("actuator '{0}' is already connected to body '{1}'")]
    AlreadyConnectedToAnotherBody(String, String),
    #[error("actuator '{0}' is already connected to that body")]
    AlreadyConnectedToThisBody(String),
}

pub trait ActuatorModel {
    type Command;
    fn update(&mut self, connection: &mut BodyConnection);
    fn result_headers(&self) -> &[&str];
    fn result_content(&self, id: u32, results: &mut ResultManager);
    /// Populates derivative with the appropriate values for the actuator state derivative
    fn state_derivative(&self, derivative: &mut SimStateVector);
    /// Initializes a vector of f64 values representing state vector for the ODE integration
    fn state_vector_init(&self) -> SimStateVector;
    /// Reads a state vector into the sim state
    fn state_vector_read(&mut self, state: &SimStateVector);
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Actuator<T>
where
    T: ActuatorModel,
{
    pub name: String,
    pub model: T,
    pub connection: Option<BodyConnection>,
    result_id: Option<u32>,
    state_id: Option<usize>,
}

impl<T> Actuator<T>
where
    T: ActuatorModel,
{
    pub fn new(name: &str, model: T) -> Self {
        Self {
            name: name.to_string(),
            model,
            connection: None,
            result_id: None,
            state_id: None,
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

    pub fn update(&mut self) {
        if let Some(connection) = &mut self.connection {
            self.model.update(connection);
        }
    }

    pub fn state_vector_init(&mut self, x0: &mut SimStates) {
        self.state_id = Some(x0.0.len());
        x0.0.push(self.model.state_vector_init());
    }

    pub fn state_vector_read(&mut self, x0: &SimStates) {
        if let Some(id) = self.state_id {
            self.model.state_vector_read(&x0.0[id as usize]);
        }
    }

    pub fn state_derivative(&self, x0: &mut SimStates) {
        if let Some(id) = self.state_id {
            self.model.state_derivative(&mut x0.0[id as usize]);
        }
    }
}

impl<T> NadirResult for Actuator<T>
where
    T: ActuatorModel,
{
    fn new_result(&mut self, results: &mut ResultManager) {
        // Define the actuator subfolder folder path
        let actuator_folder_path = results.result_path.join("actuators");

        // Check if the folder exists, if not, create it
        if !actuator_folder_path.exists() {
            std::fs::create_dir_all(&actuator_folder_path)
                .expect("Failed to create actuator folder");
        }

        // Initialize writer using the updated path
        let id = results.new_writer(
            &self.name,
            &actuator_folder_path,
            self.model.result_headers(),
        );
        self.result_id = Some(id);
    }

    fn write_result(&self, results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            self.model.result_content(id, results);
        }
    }
}

pub trait ActuatorSystem: Serialize {
    fn update(&mut self);
    fn initialize_results(&mut self, results: &mut ResultManager);
    fn write_results(&self, results: &mut ResultManager);
    fn state_vector_init(&mut self, x0: &mut SimStates);
    fn state_vector_read(&mut self, x0: &SimStates);
    fn state_derivative(&self, x0: &mut SimStates);
}

impl ActuatorSystem for () {
    fn update(&mut self) {}
    fn initialize_results(&mut self, _results: &mut ResultManager) {}
    fn write_results(&self, _results: &mut ResultManager) {}
    fn state_vector_init(&mut self, _x0: &mut SimStates) {}
    fn state_vector_read(&mut self, _x0: &SimStates) {}
    fn state_derivative(&self, _x0: &mut SimStates) {}
}
