use indexmap::IndexMap;
use nadir_result::{NadirResult, ResultManager};
use reaction_wheel::ReactionWheel;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;
use thiserror::Error;
use transforms::Transform;

use crate::{
    body::{Body, BodyConnection},
    solver::{SimStateVector, SimStates},
    system::Id,
};

pub mod reaction_wheel;

#[derive(Debug, Clone, Error)]
pub enum ActuatorErrors {
    #[error("actuator '{0}' is already connected to body '{1}'")]
    AlreadyConnectedToAnotherBody(String, String),
    #[error("actuator '{0}' is already connected to that body")]
    AlreadyConnectedToThisBody(String),
}

pub trait ActuatorModel {
    fn update(&mut self, body: &mut Body, body_transform: &Transform);
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
pub struct Actuator {
    pub name: String,
    pub model: ActuatorModels,
    pub connection: Option<BodyConnection>,
    /// Id of the result writer in sys.writers
    result_id: Option<u32>,
    /// Id of state index in SimStateVector
    state_id: Option<usize>,
}

impl Actuator {
    pub fn new(name: &str, model: ActuatorModels) -> Self {
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
        body: Id,
        transform: Transform,
    ) -> Result<(), ActuatorErrors> {
        if let Some(connection) = &self.connection {
            return Err(ActuatorErrors::AlreadyConnectedToAnotherBody(
                self.name.clone(),
                connection.body.to_string(),
            ));
        }

        self.connection = Some(BodyConnection::new(body, transform));
        Ok(())
    }

    pub fn update(&mut self, bodies: &mut IndexMap<Id, Body>) {
        if let Some(connection) = &mut self.connection {
            if let Some(body) = bodies.get_mut(&connection.body) {
                self.model.update(body, &connection.transform);
            }
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

impl NadirResult for Actuator {
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ActuatorModels {
    ReactionWheel(ReactionWheel),
}

impl ActuatorModel for ActuatorModels {
    fn result_content(&self, id: u32, results: &mut ResultManager) {
        match self {
            ActuatorModels::ReactionWheel(act) => act.result_content(id, results),
        }
    }

    fn result_headers(&self) -> &[&str] {
        match self {
            ActuatorModels::ReactionWheel(act) => act.result_headers(),
        }
    }

    fn state_derivative(&self, derivative: &mut SimStateVector) {
        match self {
            ActuatorModels::ReactionWheel(act) => act.state_derivative(derivative),
        }
    }

    fn state_vector_init(&self) -> SimStateVector {
        match self {
            ActuatorModels::ReactionWheel(act) => act.state_vector_init(),
        }
    }

    fn state_vector_read(&mut self, state: &SimStateVector) {
        match self {
            ActuatorModels::ReactionWheel(act) => act.state_vector_read(state),
        }
    }

    fn update(&mut self, body: &mut Body, body_transform: &Transform) {
        match self {
            ActuatorModels::ReactionWheel(act) => act.update(body, body_transform),
        }
    }
}
