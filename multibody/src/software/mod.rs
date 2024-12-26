use serde::Serialize;

use crate::{actuator::ActuatorSystem, sensor::SensorSystem};

pub trait SoftwareSystem: Serialize {
    type Actuators: ActuatorSystem;
    type Sensors: SensorSystem;
    fn run(&mut self, sensors: &Self::Sensors, actuators: &mut Self::Actuators);
}