pub mod rate;
pub mod rate3;

use rate3::Rate3Sensor;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum SimpleSensors {
    //Angle(AngleSensor),
    //Attitude(AttitudeSensor),
    //Rate(RateSensor),
    Rate3(Rate3Sensor),
    //Position(PositionSensor),
    //Position3(Position3Sensor),
    //Velocity(VelocitySensor),
    //Velocity3(Velocity3Sensor),
}
