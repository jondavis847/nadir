use serde::Serialize;

use crate::{actuator::ActuatorSystem, sensor::SensorSystem};

pub trait SoftwareSystem: Serialize {
    type Actuators: ActuatorSystem;
    type Sensors: SensorSystem;
    fn run(&mut self, sensors: &Self::Sensors, actuators: &mut Self::Actuators);
}

// this lets the software be optional in sys
impl SoftwareSystem for () {
    type Actuators = ();
    type Sensors = ();
    fn run(&mut self, _sensors: &Self::Sensors, _actuators: &mut Self::Actuators) {}
}