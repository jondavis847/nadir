use crate::{
    actuator::{Actuator, ActuatorErrors},
    sensor::Sensor,
    HardwareBuffer,
};
use libloading::{Library, Symbol};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SoftwareErrors {
    #[error("{0}")]
    Actuator(#[from] ActuatorErrors),
    #[error("Failed to load library: {0}")]
    LibraryLoadError(String),
    #[error("Failed to load symbol: {0}")]
    MissingEntryPoint(String),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Software {
    pub name: String,
    pub lib: String,
    pub sensor_indices: Vec<usize>,
    pub actuator_indices: Vec<usize>,
}

// Define the C FFI function type
pub type FnC = unsafe extern "C" fn(
    sensors: *const HardwareBuffer,
    sensor_count: usize,
    commands: *mut HardwareBuffer,
    actuator_count: usize,
);

#[derive(Debug)]
pub struct SoftwareSim {
    entry: FnC,
    sensor_indices: Vec<usize>,
    actuator_indices: Vec<usize>,
    sensor_telemetry_cache: Vec<HardwareBuffer>,
    actuator_command_cache: Vec<HardwareBuffer>,
    _lib: Library, // to keep it alive
}

impl SoftwareSim {
    pub fn new(
        entry: FnC,
        sensor_indices: Vec<usize>,
        actuator_indices: Vec<usize>,
        lib: Library,
    ) -> Self {
        let sensor_telemetry_cache = vec![HardwareBuffer::new(); sensor_indices.len()];
        let actuator_command_cache = vec![HardwareBuffer::new(); actuator_indices.len()];

        Self {
            entry,
            sensor_indices,
            actuator_indices,
            sensor_telemetry_cache,
            actuator_command_cache,
            _lib: lib,
        }
    }

    pub fn step(
        &mut self,
        sensors: &[Sensor],
        actuators: &mut [Actuator],
    ) -> Result<(), SoftwareErrors> {
        // write sensor telemetry to cache
        for &i in &self.sensor_indices {
            self.sensor_telemetry_cache[i].write_bytes(sensors[i].telemetry_buffer.as_bytes());
        }

        // Run C software logic
        unsafe {
            (self.entry)(
                self.sensor_telemetry_cache.as_ptr(),
                self.sensor_telemetry_cache.len(),
                self.actuator_command_cache.as_mut_ptr(),
                self.actuator_command_cache.capacity(),
            );
        }

        // read actuator commands
        for (i, &a) in self.actuator_indices.iter().enumerate() {
            actuators[a].read_command(&self.actuator_command_cache[i])?;
        }
        Ok(())
    }
}

impl TryFrom<&Software> for SoftwareSim {
    type Error = SoftwareErrors;
    fn try_from(soft: &Software) -> Result<Self, SoftwareErrors> {
        let path = soft.lib.as_str();
        let lib = unsafe {
            Library::new(path).unwrap_or_else(|e| panic!("Failed to load C cdylib {}: {}", path, e))
        };

        // Look up the entry point fns
        let step_fn: Symbol<FnC> = unsafe {
            lib.get(b"step")
                .unwrap_or_else(|e| panic!("Missing step in {}: {}", path, e))
        };

        let entry = *step_fn;

        Ok(SoftwareSim::new(
            entry,
            soft.sensor_indices.clone(),
            soft.actuator_indices.clone(),
            lib,
        ))
    }
}
