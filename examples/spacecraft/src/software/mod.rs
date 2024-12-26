use actuators::ActuatorFsw;
use control::ControlFsw;
use guidance::GuidanceFsw;
use multibody::software::SoftwareSystem;
use navigation::NavigationFsw;
use sensors::SensorFsw;
use serde::{Deserialize, Serialize};
use crate::hardware::{actuators::SpacecraftActuators, sensors::SpacecraftSensors};

mod actuators;
mod control;
mod guidance;
mod navigation;
mod sensors;

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct SpacecraftFsw {
    sensors: SensorFsw,
    navigation: NavigationFsw,
    guidance: GuidanceFsw, 
    control: ControlFsw,    
    actuators: ActuatorFsw,
}

impl SoftwareSystem for SpacecraftFsw {
    type Actuators = SpacecraftActuators;
    type Sensors = SpacecraftSensors;

    fn run(&mut self, sensors: &Self::Sensors, actuators: &mut Self::Actuators) {        
        self.sensors.run(sensors);
        self.navigation.run(&self.sensors);
        self.guidance.run(&self.navigation);
        self.control.run(&self.navigation, &self.guidance);
        self.actuators.run(&self.control, actuators);        
    }   
}