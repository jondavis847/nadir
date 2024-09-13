pub mod noise;
pub mod simple;

use crate::{
    body::{BodyConnection, BodySim},
    result::{MultibodyResultTrait,ResultEntry},
};
use serde::{Deserialize, Serialize};
use simple::{SimpleSensor, SimpleSensorResult};
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

    pub fn update(&mut self, body: &BodySim) {
        if let Some(connection) = &self.connection {
            self.model.update(body, connection);
        } else {
            panic!("no sensor connection found. should not be possible, should be caught in system validation.")
        }
    }
}

impl SensorTrait for Sensor {
    fn initialize_result(&self) -> SensorResult {
        match &self.model {
            SensorModel::Simple(sensor) => sensor.initialize_result(),
        }
    }
    fn update(&mut self, body: &BodySim, connection: &BodyConnection) {
        match &mut self.model {
            SensorModel::Simple(sensor) => sensor.update(body, connection),
        }
    }
}

pub trait SensorTrait {
    fn initialize_result(&self) -> SensorResult;
    fn update(&mut self, body: &BodySim, connection: &BodyConnection);
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SensorModel {
    //Custom,
    Simple(SimpleSensor),
}

impl SensorTrait for SensorModel {
    fn initialize_result(&self) -> SensorResult {
        match self {
            SensorModel::Simple(sensor) => sensor.initialize_result(),
        }
    }
    fn update(&mut self, body: &BodySim, connection: &BodyConnection) {
        match self {
            SensorModel::Simple(sensor) => sensor.update(body, connection),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SensorResult {
    Simple(SimpleSensorResult),
    //Custom(CustomSensorResult), TODO
}

impl SensorResult {
    pub fn update(&mut self, sensor: &Sensor) {
        match (self, &sensor.model) {
            (SensorResult::Simple(result), SensorModel::Simple(sensor)) => result.update(sensor),
           // _ => unreachable!("invalid combo"),
        }
    }
}

impl MultibodyResultTrait for SensorResult {
    fn get_state_names(&self) -> Vec<&'static str> {
        match self {
            SensorResult::Simple(result) => result.get_state_names(),
        }
    }
    fn get_result_entry(&self) -> ResultEntry {
        match self {
            SensorResult::Simple(result) => result.get_result_entry(),
        }
    }
}
