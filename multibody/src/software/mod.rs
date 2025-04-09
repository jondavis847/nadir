use crate::{actuator::Actuator, sensor::Sensor, HardwareBuffer};

#[repr(C)]
pub struct CInterface {
    pub data_ptr: *const u8,
    pub data_len: usize,
}

pub trait RustSoftware {
    fn step(
        &mut self,
        sensor_telemetry: &[HardwareBuffer],
        actuator_commands: &mut [HardwareBuffer],
    );
}

pub enum SoftwareInterface {
    Rust(Box<dyn RustSoftware>),
    C(FnC),
}

// Define the C FFI function type
pub type FnC = unsafe extern "C" fn(
    sensors: *const HardwareBuffer,
    sensor_count: usize,
    commands: *mut HardwareBuffer,
    actuator_count: usize,
);

pub struct Software {
    interface: SoftwareInterface,
    sensor_indices: Vec<usize>,
    actuator_indices: Vec<usize>,
    sensor_telemetry_cache: Vec<HardwareBuffer>,
    actuator_command_cache: Vec<HardwareBuffer>,
}

impl Software {
    pub fn new(
        interface: SoftwareInterface,
        sensor_indices: Vec<usize>,
        actuator_indices: Vec<usize>,
    ) -> Self {
        let sensor_telemetry_cache = vec![HardwareBuffer::new(); sensor_indices.len()];
        let actuator_command_cache = vec![HardwareBuffer::new(); actuator_indices.len()];

        Self {
            interface,
            sensor_indices,
            actuator_indices,
            sensor_telemetry_cache,
            actuator_command_cache,
        }
    }
}

impl Software {
    pub fn step(&mut self, sensors: &[Sensor], actuators: &mut [Actuator]) {
        // write sensor telemetry to cache
        for &i in &self.sensor_indices {
            self.sensor_telemetry_cache[i].write_bytes(sensors[i].telemetry_buffer.as_bytes());
        }

        match &mut self.interface {
            SoftwareInterface::Rust(fsw) => {
                // run Rust software logic
                fsw.step(
                    self.sensor_telemetry_cache.as_slice(),
                    self.actuator_command_cache.as_mut_slice(),
                );
            }
            SoftwareInterface::C(c_fn) => {
                // Run C software logic
                unsafe {
                    (c_fn)(
                        self.sensor_telemetry_cache.as_ptr(),
                        self.sensor_telemetry_cache.len(),
                        self.actuator_command_cache.as_mut_ptr(),
                        self.actuator_command_cache.capacity(),
                    );
                }
            }
        }

        // read actuator commands
        for (i, &a) in self.actuator_indices.iter().enumerate() {
            actuators[a].read_command(&self.actuator_command_cache[i]);
        }
    }
}
