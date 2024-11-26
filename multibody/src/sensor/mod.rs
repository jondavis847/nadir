pub mod noise;
pub mod simple;

use crate::{
    body::{Body, BodyConnection, BodySim}, result::{MultibodyResultTrait, ResultEntry}, MultibodyErrors, MultibodyTrait
};
use serde::{Deserialize, Serialize};
use simple::{SimpleSensor, SimpleSensorResult};
use transforms::Transform;
use uuid::Uuid;

#[derive(Debug, Clone)]
pub enum SensorErrors {
    AlreadyConnected,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Sensor {
    name: String,
    model: SensorModel,
    connection: Option<BodyConnection>,
    id: Uuid,
}

impl Sensor {
    pub fn connect_to_body(
        &mut self,
        body: &mut Body,
        transform: Transform,
    ) -> Result<(), SensorErrors> {
        if self.connection.is_some() {
            return Err(SensorErrors::AlreadyConnected);
        }

        if !body.sensors.contains(&self.id) {
            body.sensors.push(self.id);
        }

        self.connection = Some(BodyConnection::new(body.id, transform));
        Ok(())
    }

    pub fn get_model(&self) -> &SensorModel {
        &self.model
    }

    pub fn new(name: String, model: SensorModel) -> Self {
        Self {
            name,
            model,
            connection: None,
            id: Uuid::new_v4(),
        }
    }

    pub fn set_model(&mut self, model: SensorModel) {
        self.model = model;
    }

    pub fn update(&mut self, body: &BodySim) {
        if let Some(connection) = &self.connection {
            self.model.update(body, connection);
        } else {
            unreachable!("no sensor connection found. should not be possible, should be caught in system validation.")
        }
    }
}

impl MultibodyTrait for Sensor {
    fn get_id(&self) -> &Uuid {
        &self.id
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
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

impl MultibodyResultTrait for SensorResult {
    type Component = Sensor;
    const STATES: &'static [&'static str] = &[];

    fn get_result_entry(&self) -> ResultEntry {
        match self {
            SensorResult::Simple(result) => result.get_result_entry(),
        }
    }

    fn get_state_names(&self) -> &'static [&'static str] {
        match self {
            SensorResult::Simple(result) => result.get_state_names(),
        }
    }

    fn get_state_value(&self, state: &str) -> Result<&Vec<f64>, MultibodyErrors> {
        match self {
            SensorResult::Simple(result) => result.get_state_value(state),
        }
    }
    fn update(&mut self, sensor: &Sensor) {
        match (self, &sensor.model) {
            (SensorResult::Simple(result), SensorModel::Simple(sensor)) => result.update(sensor),
            // _ => unreachable!("invalid combo"),
        }
    }
}
