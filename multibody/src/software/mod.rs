use crate::{
    actuator::Actuator,
    sensor::{Sensor, SensorModel},
};

#[repr(C)]
pub struct SensorData {
    pub data_ptr: *const u8,
    pub data_len: usize,
}

#[repr(C)]
pub struct ActuatorCommand {
    pub actuator_index: usize,
    pub data_ptr: *const u8,
    pub data_len: usize,
}

pub enum SoftwareInterface {
    Rust(SoftwareEntryFnRust),
    C(SoftwareEntryFnC),
}

pub type SoftwareEntryFnRust = fn(sensors: &[SensorData], commands: &mut Vec<ActuatorCommand>);

pub type SoftwareEntryFnC = unsafe extern "C" fn(
    sensors: *const SensorData,
    sensor_count: usize,
    commands: *mut ActuatorCommand,
    max_count: usize,
    out_count: *mut usize,
);

pub struct Software {
    interface: SoftwareInterface,
    sensor_indices: Vec<usize>,
    actuator_indices: Vec<usize>,
    sensor_data: Vec<SensorData>,
    actuator_commands: Vec<ActuatorCommand>,
}

impl Software {
    pub fn step(&mut self, sensors: &[Sensor], actuators: &mut [Actuator]) {
        // clear old commands
        self.actuator_commands.clear();

        // collect sensor data
        for (j, &i) in self.sensor_indices.iter().enumerate() {
            let telemetry = sensors[i].model.telemetry();
            self.sensor_data[j] = SensorData {
                data_ptr: telemetry.as_ptr(),
                data_len: telemetry.len(),
            };
        }

        match self.interface {
            SoftwareInterface::Rust(entry_fn) => {
                // run Rust software logic
                entry_fn(&self.sensor_data, &mut self.actuator_commands);
            }
            SoftwareInterface::C(entry_fn) => {
                // run C software logic
                let mut out_count: usize = 0;
                unsafe {
                    entry_fn(
                        self.sensor_data.as_ptr(),
                        self.sensor_data.len(),
                        self.actuator_commands.as_mut_ptr(),
                        self.actuator_commands.capacity(),
                        &mut out_count,
                    );
                    self.actuator_commands.set_len(out_count);
                }
            }
        }

        // apply actuator commands
        for cmd in &self.actuator_commands {
            let act = &mut actuators[cmd.actuator_index];
            unsafe {
                let bytes = std::slice::from_raw_parts(cmd.data_ptr, cmd.data_len);
                act.model.apply_command(bytes);
            }
        }
    }
}
