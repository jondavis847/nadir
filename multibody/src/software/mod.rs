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
    fn step(&mut self, sensor_telemetry: &[&[u8]], actuator_commands: &mut [&mut [u8]]);
}

pub enum SoftwareInterface {
    Rust(Box<dyn RustSoftware>),
    C(CSoftware),
}

pub struct CSoftware {
    entry_fn: CEntryFn,
    sensor_telemetry_cache: Vec<CInterface>,
    actuator_command_cache: Vec<CInterface>,
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
}

impl Software {
    pub fn step(&mut self, sensors: &[Sensor], actuators: &mut [Actuator]) {
        match &mut self.interface {
            SoftwareInterface::Rust(fsw) => {
                // Create new slices with only the elements at the specified indices
                let sensor_data: [u8] = self
                    .sensor_indices
                    .iter()
                    .map(|&idx| &sensors[idx].model.read_telemetry_bytes())
                    .collect();

                let mut selected_actuators: Vec<&mut Actuator> = self
                    .actuator_indices
                    .iter()
                    .map(|&idx| &mut actuators[idx])
                    .collect();

                // run Rust software logic
                fsw.step(&sensor_data, selected_actuators.as_mut_slice());
            }
            SoftwareInterface::C(fsw) => {
                // collect sensor data
                for (i, &s) in self.sensor_indices.iter().enumerate() {
                    fsw.sensor_telemetry_cache[i] = sensors[s].model.read_telemetry_c();
                }
                // Run C software logic
                let mut out_count: usize = 0;
                unsafe {
                    (fsw.entry_fn)(
                        fsw.sensor_telemetry_cache.as_ptr(),
                        fsw.sensor_telemetry_cache.len(),
                        fsw.actuator_command_cache.as_mut_ptr(),
                        fsw.actuator_command_cache.capacity(),
                        &mut out_count,
                    );
                    fsw.actuator_command_cache.set_len(out_count);
                }
                // apply actuator commands
                for (i, &a) in self.actuator_indices.iter().enumerate() {
                    let act = &mut actuators[a];
                    let cmd = &fsw.actuator_command_cache[i];
                    unsafe {
                        let bytes = std::slice::from_raw_parts(cmd.data_ptr, cmd.data_len);
                        act.model.write_command(bytes);
                    }
                }
            }
        }
    }
}
