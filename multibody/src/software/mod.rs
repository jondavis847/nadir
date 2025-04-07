use crate::{
    actuator::{Actuator, ActuatorModel},
    sensor::{Sensor, SensorModel},
};

#[repr(C)]
pub struct CInterface {
    pub data_ptr: *const u8,
    pub data_len: usize,
}

pub trait RustSoftware {
    fn step(&mut self, sensors: &[CInterface], actuators: &mut Vec<CInterface>);
}

pub enum SoftwareInterface {
    Rust(Box<dyn RustSoftware>),
    C(CEntryFn),
}

// Define the C FFI function type
pub type CEntryFn = unsafe extern "C" fn(
    sensors: *const CInterface,
    sensor_count: usize,
    commands: *mut CInterface,
    max_count: usize,
    out_count: *mut usize,
);

pub struct Software {
    interface: SoftwareInterface,
    sensor_indices: Vec<usize>,
    actuator_indices: Vec<usize>,
    sensor_telemetry_cache: Vec<CInterface>,
    actuator_command_cache: Vec<CInterface>,
}

impl Software {
    pub fn step(&mut self, sensors: &[Sensor], actuators: &mut [Actuator]) {
        // collect sensor data
        for (i, &s) in self.sensor_indices.iter().enumerate() {
            self.sensor_telemetry_cache[i] = sensors[s].model.read_telemetry();
        }

        match &mut self.interface {
            SoftwareInterface::Rust(fsw) => {
                // run Rust software logic
                fsw.step(
                    &self.sensor_telemetry_cache,
                    &mut self.actuator_command_cache,
                );
            }
            SoftwareInterface::C(entry_fn) => {
                // Run C software logic
                let mut out_count: usize = 0;
                unsafe {
                    entry_fn(
                        self.sensor_telemetry_cache.as_ptr(),
                        self.sensor_telemetry_cache.len(),
                        self.actuator_command_cache.as_mut_ptr(),
                        self.actuator_command_cache.capacity(),
                        &mut out_count,
                    );
                    self.actuator_command_cache.set_len(out_count);
                }
            }
        }

        // apply actuator commands
        for (i, &a) in self.actuator_indices.iter().enumerate() {
            let act = &mut actuators[a];
            let cmd = &self.actuator_command_cache[i];
            unsafe {
                let bytes = std::slice::from_raw_parts(cmd.data_ptr, cmd.data_len);
                act.model.write_command(bytes);
            }
        }
    }
}
