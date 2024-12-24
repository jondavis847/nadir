use multibody::{sensor::Sensor, software::SoftwareSystem};
use sensor_processing::SensorProcessing;
use serde::{Deserialize, Serialize};

use crate::hardware::SpacecraftSensors;

mod sensor_processing;

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct SpacecraftSoftware {
    sp: SensorProcessing,
}

impl SoftwareSystem for SpacecraftSoftware {
    type Actuators = ();
    type Sensors = SpacecraftSensors;
    fn run(&mut self, sensors: &Self::Sensors, actuators: &mut Self::Actuators) {
        self.sp.run(&sensors);
    }
}
