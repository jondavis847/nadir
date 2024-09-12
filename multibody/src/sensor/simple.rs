pub mod rate;
pub mod rate3;

use crate::body::{BodyConnection,BodySim};
use rate::RateSensor;
use rate3::Rate3Sensor;
use serde::{Deserialize, Serialize};
use super::SensorTrait;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SimpleSensors {
    //Angle(AngleSensor),
    //Attitude(AttitudeSensor),
    Rate(RateSensor),
    Rate3(Rate3Sensor),
    //Position(PositionSensor),
    //Position3(Position3Sensor),
    //Velocity(VelocitySensor),
    //Velocity3(Velocity3Sensor),
}

impl SensorTrait for SimpleSensors {
    fn update(&mut self, body: &BodySim, connection: &BodyConnection) {
        match self {
            SimpleSensors::Rate(sensor) => sensor.update(body,connection),
            SimpleSensors::Rate3(sensor) => sensor.update(body,connection),
        }
    }
}