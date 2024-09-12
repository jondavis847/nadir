pub mod rate;
pub mod rate3;

use crate::{body::{BodyConnection,BodySim},result::MultibodyResultTrait};
use rate::{RateSensor, RateSensorResult};
use rate3::{Rate3Sensor, Rate3SensorResult};
use serde::{Deserialize, Serialize};
use super::SensorTrait;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SimpleSensor {
    //Angle(AngleSensor),
    //Attitude(AttitudeSensor),
    Rate(RateSensor),
    Rate3(Rate3Sensor),
    //Position(PositionSensor),
    //Position3(Position3Sensor),
    //Velocity(VelocitySensor),
    //Velocity3(Velocity3Sensor),
}

impl SensorTrait for SimpleSensor {
    fn update(&mut self, body: &BodySim, connection: &BodyConnection) {
        match self {
            SimpleSensor::Rate(sensor) => sensor.update(body,connection),
            SimpleSensor::Rate3(sensor) => sensor.update(body,connection),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SimpleSensorResult {
    Rate(RateSensorResult),
    Rate3(Rate3SensorResult),    
}

impl MultibodyResultTrait for SimpleSensorResult {
    fn get_state_names(&self) -> Vec<&'static str> {
        match self {
            SimpleSensorResult::Rate(result) => result.get_state_names(),
            SimpleSensorResult::Rate3(result) => result.get_state_names(),
        }
    }
}

