use crate::{
    HardwareBuffer,
    actuator::{Actuator, ActuatorErrors},
    sensor::Sensor,
};
use libloading::{Library, Symbol};
use nadir_result::ResultManager;
use serde::{Deserialize, Serialize};
use std::{ffi::c_void, path::Path};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum SoftwareErrors {
    #[error("{0}")]
    Actuator(#[from] ActuatorErrors),
    #[error("Failed to load library: {0}")]
    LibraryLoadError(String),
    #[error("Failed to load symbol: {0}")]
    MissingEntryPoint(String),
    #[error("Software initialization failed with code: {0}")]
    InitializationError(i32),
    #[error("Software execution failed with code: {0}")]
    ExecutionError(i32),
    #[error("Software step returned null state pointer")]
    NullStatePointer,
    #[error("Invalid pointer passed to software")]
    InvalidPointer,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Software {
    pub name: String,
    pub lib_path: String,
    pub sensor_indices: Vec<usize>,
    pub actuator_indices: Vec<usize>,
}

impl Software {
    pub fn new<P: AsRef<Path>>(name: &str, lib_path: P) -> Self {
        Self {
            name: name.to_string(),
            lib_path: lib_path
                .as_ref()
                .to_string_lossy()
                .to_string(),
            sensor_indices: Vec::new(),
            actuator_indices: Vec::new(),
        }
    }

    pub fn with_sensor_indices(mut self, indices: Vec<usize>) -> Self {
        self.sensor_indices = indices;
        self
    }

    pub fn with_actuator_indices(mut self, indices: Vec<usize>) -> Self {
        self.actuator_indices = indices;
        self
    }
}

// Enhanced FFI function types with proper state handling and error codes
pub type InitFnC = unsafe extern "C" fn() -> *mut std::ffi::c_void;
pub type InitResultsFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32;
pub type WriteResultsFn = unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32;
pub type StepFnC = unsafe extern "C" fn(
    software_state: *mut std::ffi::c_void,
    sensors: *const HardwareBuffer,
    sensor_count: usize,
    commands: *mut HardwareBuffer,
    actuator_count: usize,
) -> i32;
pub type CleanupFnC = unsafe extern "C" fn(software_state: *mut std::ffi::c_void);

#[derive(Debug)]
pub struct SoftwareSim {
    step_fn: StepFnC,
    init_results_fn: InitResultsFn,
    write_results_fn: WriteResultsFn,
    cleanup_fn: CleanupFnC,
    software_state: *mut std::ffi::c_void,
    sensor_indices: Vec<usize>,
    actuator_indices: Vec<usize>,
    sensor_telemetry_cache: Vec<HardwareBuffer>,
    actuator_command_cache: Vec<HardwareBuffer>,
    _lib: Library, // to keep it alive
}

impl SoftwareSim {
    pub fn new<P: AsRef<Path>>(
        lib_path: P,
        sensor_indices: Vec<usize>,
        actuator_indices: Vec<usize>,
    ) -> Result<Self, SoftwareErrors> {
        // Load the dynamic library
        let lib = unsafe {
            Library::new(lib_path.as_ref()).map_err(|e| {
                SoftwareErrors::LibraryLoadError(format!(
                    "Failed to load library {}: {}",
                    lib_path
                        .as_ref()
                        .display(),
                    e
                ))
            })?
        };

        // Get the initialization function
        let init_fn: Symbol<InitFnC> = unsafe {
            lib.get(b"initialize_software")
                .map_err(|e| {
                    SoftwareErrors::MissingEntryPoint(format!(
                        "Missing initialize_software in {}: {}",
                        lib_path
                            .as_ref()
                            .display(),
                        e
                    ))
                })?
        };

        // Get the initialization results function
        let init_results_fn: Symbol<InitResultsFn> = unsafe {
            lib.get(b"initialize_results")
                .map_err(|e| {
                    SoftwareErrors::MissingEntryPoint(format!(
                        "Missing initialize_results in {}: {}",
                        lib_path
                            .as_ref()
                            .display(),
                        e
                    ))
                })?
        };

        // Get the write results function
        let write_results_fn: Symbol<WriteResultsFn> = unsafe {
            lib.get(b"write_results")
                .map_err(|e| {
                    SoftwareErrors::MissingEntryPoint(format!(
                        "Missing write_results in {}: {}",
                        lib_path
                            .as_ref()
                            .display(),
                        e
                    ))
                })?
        };

        // Get the step function
        let step_fn: Symbol<StepFnC> = unsafe {
            lib.get(b"step_software")
                .map_err(|e| {
                    SoftwareErrors::MissingEntryPoint(format!(
                        "Missing step_software in {}: {}",
                        lib_path
                            .as_ref()
                            .display(),
                        e
                    ))
                })?
        };

        // Get the cleanup function
        let cleanup_fn: Symbol<CleanupFnC> = unsafe {
            lib.get(b"destroy_software")
                .map_err(|e| {
                    SoftwareErrors::MissingEntryPoint(format!(
                        "Missing destroy_software in {}: {}",
                        lib_path
                            .as_ref()
                            .display(),
                        e
                    ))
                })?
        };

        // Initialize the software state
        let software_state = unsafe { init_fn() };
        if software_state.is_null() {
            return Err(SoftwareErrors::NullStatePointer);
        }

        // Prepare caches
        let sensor_telemetry_cache = vec![HardwareBuffer::new(); sensor_indices.len()];
        let actuator_command_cache = vec![HardwareBuffer::new(); actuator_indices.len()];

        Ok(Self {
            step_fn: *step_fn,
            init_results_fn: *init_results_fn,
            write_results_fn: *write_results_fn,
            cleanup_fn: *cleanup_fn,
            software_state,
            sensor_indices,
            actuator_indices,
            sensor_telemetry_cache,
            actuator_command_cache,
            _lib: lib,
        })
    }

    pub fn step(
        &mut self,
        sensors: &[Sensor],
        actuators: &mut [Actuator],
    ) -> Result<(), SoftwareErrors> {
        // Check for valid state
        if self
            .software_state
            .is_null()
        {
            return Err(SoftwareErrors::NullStatePointer);
        }

        // Copy sensor telemetry to cache
        for (cache_idx, &sensor_idx) in self
            .sensor_indices
            .iter()
            .enumerate()
        {
            if sensor_idx < sensors.len() {
                self.sensor_telemetry_cache[cache_idx].write_bytes(
                    sensors[sensor_idx]
                        .telemetry_buffer
                        .as_bytes(),
                );
            }
        }

        // Run software step with persistent state
        let result = unsafe {
            (self.step_fn)(
                self.software_state,
                self.sensor_telemetry_cache
                    .as_ptr(),
                self.sensor_telemetry_cache
                    .len(),
                self.actuator_command_cache
                    .as_mut_ptr(),
                self.actuator_command_cache
                    .len(),
            )
        };

        // Check for errors
        if result != 0 {
            return Err(SoftwareErrors::ExecutionError(result));
        }

        // Read actuator commands
        for (cache_idx, &actuator_idx) in self
            .actuator_indices
            .iter()
            .enumerate()
        {
            if actuator_idx < actuators.len() {
                actuators[actuator_idx].read_command(&self.actuator_command_cache[cache_idx])?;
            }
        }

        Ok(())
    }

    pub fn initialize_results(&self, results: &mut ResultManager) -> Result<(), SoftwareErrors> {
        // Ensure the software state is valid
        if self
            .software_state
            .is_null()
        {
            return Err(SoftwareErrors::NullStatePointer);
        }

        // Convert the results reference to a raw pointer
        let results_ptr = results as *mut _ as *mut c_void;

        // Call the FFI function
        let status = unsafe {
            (self.init_results_fn)(
                self.software_state,
                results_ptr,
            )
        };

        if status != 0 {
            Err(SoftwareErrors::ExecutionError(status))
        } else {
            Ok(())
        }
    }

    pub fn write_results(&self, results: &mut ResultManager) -> Result<(), SoftwareErrors> {
        // Ensure the software state is valid
        if self
            .software_state
            .is_null()
        {
            return Err(SoftwareErrors::NullStatePointer);
        }

        // Convert the results reference to a raw pointer
        let results_ptr = results as *mut _ as *mut c_void;

        // Call the FFI function
        let status = unsafe {
            (self.write_results_fn)(
                self.software_state,
                results_ptr,
            )
        };

        if status != 0 {
            Err(SoftwareErrors::ExecutionError(status))
        } else {
            Ok(())
        }
    }
}

// Properly clean up resources when SoftwareSim is dropped
impl Drop for SoftwareSim {
    fn drop(&mut self) {
        if !self
            .software_state
            .is_null()
        {
            unsafe { (self.cleanup_fn)(self.software_state) };
            self.software_state = std::ptr::null_mut();
        }
    }
}

impl TryFrom<&Software> for SoftwareSim {
    type Error = SoftwareErrors;

    fn try_from(soft: &Software) -> Result<Self, Self::Error> {
        SoftwareSim::new(
            &soft.lib_path,
            soft.sensor_indices
                .clone(),
            soft.actuator_indices
                .clone(),
        )
    }
}
