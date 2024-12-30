use nadir_result::ResultManager;
use serde::Serialize;

use crate::{actuator::ActuatorSystem, sensor::SensorSystem};

pub trait SoftwareSystem: Serialize {
    type Actuators: ActuatorSystem;
    type Sensors: SensorSystem;
    fn run(&mut self, sensors: &Self::Sensors, actuators: &mut Self::Actuators);
    fn initialize_results(&mut self, results: &mut ResultManager);
    fn write_results(&self, results: &mut ResultManager);
}

impl SoftwareSystem for () {
    type Actuators = ();
    type Sensors = ();
    fn run(&mut self, _sensors: &Self::Sensors, _actuators: &mut Self::Actuators) {}
    fn initialize_results(&mut self, _results: &mut ResultManager) {}
    fn write_results(&self, _results: &mut ResultManager) {}
}
