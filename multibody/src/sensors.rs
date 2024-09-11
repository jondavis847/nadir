pub mod noise;
pub mod simple;

use crate::body::{BodyConnection, BodySim};
use serde::{Deserialize, Serialize};
use simple::SimpleSensors;
use uuid::Uuid;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sensor {
    name: String,
    model: SensorModel,
    connection: Option<BodyConnection>,
    id: Uuid,
}

impl Sensor {
    pub fn new(name: String, model: SensorModel) -> Self {
        Self {
            name,
            model,
            connection: None,
            id: Uuid::new_v4(),
        }
    }

    pub fn get_id(&self) -> &Uuid {
        &self.id
    }

    pub fn get_name(&self) -> &str {
        &self.name
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SensorModel {
    Custom,
    Simple(SimpleSensors),
}

pub trait SensorTrait {
    fn update(&mut self, connection: &BodyConnection, body: &crate::body::BodySim);
}
