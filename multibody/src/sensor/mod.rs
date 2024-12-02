pub mod noise;

use crate::{
    body::{Body, BodyConnection},
    result::MultibodyResultTrait,
};
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

#[typetag::serde]
pub trait SensorModel: CloneSensorModel + Debug + MultibodyResultTrait {
    fn update(&mut self, body: &Body, connection: &BodyConnection);
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
        body: &mut Body,
        transform: Transform,
    ) -> Result<(), SensorErrors> {
        if let Some(connection) = &self.connection {
            return Err(SensorErrors::AlreadyConnectedToAnotherBody(
                self.name.clone(),
                connection.body_id.clone(),
            ));
        }

        if body.sensors.contains(&self.name) {
            return Err(SensorErrors::AlreadyConnectedToThisBody(self.name.clone()));
        } else {
            body.sensors.push(self.name.clone());
        }

        self.connection = Some(BodyConnection::new(body.name.clone(), transform));
        Ok(())
    }

    pub fn new(name: String, model: impl SensorModel + 'static) -> Self {
        Self {
            name,
            model: Box::new(model),
            connection: None,
        }
    }

    pub fn update(&mut self, body: &Body) {
        if let Some(connection) = &self.connection {
            self.model.update(body, connection);
        } else {
            unreachable!("no sensor connection found. should not be possible, should be caught in system validation.")
        }
    }
}
