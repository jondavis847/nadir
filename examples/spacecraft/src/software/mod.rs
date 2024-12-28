use actuators::ActuatorFsw;
use control::ControlFsw;
use guidance::GuidanceFsw;
use multibody::software::SoftwareSystem;
use nadir_result::NadirResult;
use navigation::NavigationFsw;
use sensors::SensorFsw;
use serde::{Deserialize, Serialize};
use crate::hardware::{actuators::SpacecraftActuators, sensors::SpacecraftSensors};

pub mod actuators;
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

    fn initialize_results(&mut self, results: &mut nadir_result::ResultManager) {
        self.sensors.initialize_results(results);
        self.navigation.initialize_results(results);
        self.guidance.new_result(results);
        self.control.new_result(results);
        self.actuators.initialize_results(results);
    }

    fn write_results(&self, results: &mut nadir_result::ResultManager) {
        self.sensors.write_results(results);
        self.navigation.write_results(results);
        self.guidance.write_result(results);
        self.control.write_result(results);
        self.actuators.write_results(results);
    }
}