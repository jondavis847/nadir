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
