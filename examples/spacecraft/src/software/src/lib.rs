use actuators::ActuatorFsw;
use control::ControlFsw;
use guidance::GuidanceFsw;
use multibody::HardwareBuffer;
use nadir_result::NadirResult;
use navigation::NavigationFsw;
use sensors::SensorFsw;

pub mod actuators;
mod control;
mod guidance;
mod navigation;
mod sensors;

#[derive(Debug, Default)]
pub struct SpacecraftFsw {
    sensors: SensorFsw,
    navigation: NavigationFsw,
    guidance: GuidanceFsw,
    control: ControlFsw,
    actuators: ActuatorFsw,
}

impl SpacecraftFsw {
    fn step(&mut self, sensor_buffers: &[HardwareBuffer], actuator_buffers: &mut [HardwareBuffer]) {
        self.sensors.read_buffers(sensor_buffers);
        self.sensors.run();
        self.navigation.run(&self.sensors);
        self.guidance.run(&self.navigation);
        self.control.run(&self.navigation, &self.guidance);
        self.actuators.run(&self.control);
        self.actuators.write_buffers(actuator_buffers);
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
