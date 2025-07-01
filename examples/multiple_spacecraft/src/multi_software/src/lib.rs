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
        self.sensors
            .read_buffers(sensor_buffers);
        self.sensors
            .run();
        self.navigation
            .run(&self.sensors);
        self.guidance
            .run(&self.navigation);
        self.control
            .run(
                &self.navigation,
                &self.guidance,
            );
        self.actuators
            .run(&self.control);
        self.actuators
            .write_buffers(actuator_buffers);
    }

    fn initialize_results(&mut self, results: &mut nadir_result::ResultManager) {
        self.sensors
            .initialize_results(results);
        self.navigation
            .initialize_results(results);
        self.guidance
            .new_result(results);
        self.control
            .new_result(results);
        self.actuators
            .initialize_results(results);
    }

    fn write_results(&self, results: &mut nadir_result::ResultManager) {
        self.sensors
            .write_results(results);
        self.navigation
            .write_results(results);
        self.guidance
            .write_result(results);
        self.control
            .write_result(results);
        self.actuators
            .write_results(results);
    }
}

// Export FFI-compatible functions for dynamic loading
#[no_mangle]
pub extern "C" fn initialize_software() -> *mut std::ffi::c_void {
    let software = Box::new(SpacecraftFsw::default());
    Box::into_raw(software) as *mut std::ffi::c_void
}

#[no_mangle]
pub extern "C" fn step_software(
    software_ptr: *mut std::ffi::c_void,
    sensor_buffers_ptr: *const HardwareBuffer,
    sensor_buffer_count: usize,
    actuator_buffers_ptr: *mut HardwareBuffer,
    actuator_buffer_count: usize,
) -> i32 {
    // Safety checks
    if software_ptr.is_null()
        || (sensor_buffers_ptr.is_null() && sensor_buffer_count > 0)
        || (actuator_buffers_ptr.is_null() && actuator_buffer_count > 0)
    {
        return -1; // Error code for null pointer
    }

    // Convert raw pointer to reference to our SpacecraftFsw
    let software = unsafe { &mut *(software_ptr as *mut SpacecraftFsw) };

    // Create Rust slices from the raw pointers
    let sensor_buffers = unsafe {
        std::slice::from_raw_parts(
            sensor_buffers_ptr,
            sensor_buffer_count,
        )
    };

    let actuator_buffers = unsafe {
        std::slice::from_raw_parts_mut(
            actuator_buffers_ptr,
            actuator_buffer_count,
        )
    };

    // Call the step method
    software.step(
        sensor_buffers,
        actuator_buffers,
    );

    0 // Success code
}

#[no_mangle]
pub extern "C" fn initialize_results(
    software_ptr: *mut std::ffi::c_void,
    results_ptr: *mut std::ffi::c_void,
) -> i32 {
    if software_ptr.is_null() || results_ptr.is_null() {
        return -1;
    }

    let software = unsafe { &mut *(software_ptr as *mut SpacecraftFsw) };
    let results = unsafe { &mut *(results_ptr as *mut nadir_result::ResultManager) };

    software.initialize_results(results);
    0
}

#[no_mangle]
pub extern "C" fn write_results(
    software_ptr: *mut std::ffi::c_void,
    results_ptr: *mut std::ffi::c_void,
) -> i32 {
    if software_ptr.is_null() || results_ptr.is_null() {
        return -1;
    }

    let software = unsafe { &*(software_ptr as *const SpacecraftFsw) };
    let results = unsafe { &mut *(results_ptr as *mut nadir_result::ResultManager) };

    software.write_results(results);
    0
}

#[no_mangle]
pub extern "C" fn destroy_software(software_ptr: *mut std::ffi::c_void) {
    if !software_ptr.is_null() {
        unsafe {
            // Convert the raw pointer back to a Box and drop it
            let _ = Box::from_raw(software_ptr as *mut SpacecraftFsw);
        }
    }
}
