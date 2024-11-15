pub mod rate;
pub mod rate3;
pub mod star_tracker;

use super::SensorTrait;
use crate::{
    body::{BodyConnection, BodySim},
    result::MultibodyResultTrait, MultibodyErrors,
};
use rate::{RateSensor, RateSensorResult};
use rate3::{Rate3Sensor, Rate3SensorResult};
use serde::{Deserialize, Serialize};
use star_tracker::StarTrackerResult;

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
    fn initialize_result(&self) -> super::SensorResult {
        match self {
            SimpleSensor::Rate(sensor) => sensor.initialize_result(),
            SimpleSensor::Rate3(sensor) => sensor.initialize_result(),
        }
    }

    fn update(&mut self, body: &BodySim, connection: &BodyConnection) {
        match self {
            SimpleSensor::Rate(sensor) => sensor.update(body, connection),
            SimpleSensor::Rate3(sensor) => sensor.update(body, connection),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum SimpleSensorResult {
    Rate(RateSensorResult),
    Rate3(Rate3SensorResult),
    StarTracker(StarTrackerResult),
}

impl SimpleSensorResult {}

impl MultibodyResultTrait for SimpleSensorResult {
    type Component = SimpleSensor;
    const STATES: &'static [&'static str] = &[];

    fn get_result_entry(&self) -> crate::result::ResultEntry {
        match self {
            SimpleSensorResult::Rate(result) => result.get_result_entry(),
            SimpleSensorResult::Rate3(result) => result.get_result_entry(),
            SimpleSensorResult::StarTracker(result) => result.get_result_entry(),
        }
    }
    fn get_state_names(&self) -> &'static [&'static str] {
        match self {
            SimpleSensorResult::Rate(result) => result.get_state_names(),
            SimpleSensorResult::Rate3(result) => result.get_state_names(),
            SimpleSensorResult::StarTracker(result) => result.get_state_names(),
        }
    }
    fn get_state_value(&self, state: &str) -> Result<&Vec<f64>, MultibodyErrors> {
        match self {
            SimpleSensorResult::Rate(result) => result.get_state_value(state),
            SimpleSensorResult::Rate3(result) => result.get_state_value(state),
            SimpleSensorResult::StarTracker(result) => result.get_state_value(state),
        }
    }
    fn update(&mut self, sensor: &SimpleSensor) {
        match (self, sensor) {
            (SimpleSensorResult::Rate(result), SimpleSensor::Rate(sensor)) => result.update(sensor),
            (SimpleSensorResult::Rate3(result), SimpleSensor::Rate3(sensor)) => {
                result.update(sensor)
            }
            _ => unreachable!("invalid combo"),
        }
    }
}
