pub mod actuator;
pub mod algorithms;
pub mod base;
pub mod body;
pub mod joint;
pub mod mechanism;
pub mod sensor;
pub mod software;
pub mod solver;
pub mod system;

use actuator::ActuatorErrors;
use base::BaseErrors;
use body::BodyErrors;
use bytemuck::{bytes_of, checked::from_bytes, Pod, Zeroable};
use celestial::CelestialErrors;

use joint::JointErrors;
use sensor::SensorErrors;
use software::CInterface;
use spice::SpiceErrors;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum MultibodyErrors {
    #[error("{0}")]
    ActuatorErrors(#[from] ActuatorErrors),
    #[error("actuator '{0}' is not connected to a body")]
    ActuatorMissingBody(String),
    #[error("{0}")]
    BaseErrors(#[from] BaseErrors),
    #[error("base does not have any outer joints")]
    BaseMissingOuterJoint,
    #[error("could not find body '{0}' in the system")]
    BodyNotFound(String),
    #[error("{0}")]
    Body(#[from] BodyErrors),
    #[error("body '{0}' does not have an inner joint")]
    BodyMissingInnerJoint(String),
    #[error("base cannot be deleted")]
    CantDeleteBase,
    #[error("{0}")]
    CelestialErrors(#[from] CelestialErrors),
    #[error("could not find component {0} in the system")]
    ComponentNotFound(String),
    #[error("could not find state '{0}' for component")]
    ComponentStateNotFound(String),
    #[error("sim dt cannot be 0.0")]
    DtCantBeZero,
    #[error("invalid connection")]
    InvalidConnection,
    #[error("{0}")]
    JointErrors(#[from] JointErrors),
    #[error("joint '{0}' must have an inner body")]
    JointMissingInnerBody(String),
    #[error("joint '{0}' must have an outer body")]
    JointMissingOuterBody(String),
    #[error("could not find joint '{0}' in system")]
    JointNotFound(String),
    #[error("the name '{0}' is already taken")]
    NameTaken(String),
    #[error("could not find transform")]
    NoTransformFound,
    #[error("{0}")]
    SensorErrors(#[from] SensorErrors),
    #[error("sensor '{0}' is not connected to a body")]
    SensorMissingBody(String),
    #[error("{0}")]
    SpiceErrors(#[from] SpiceErrors),
}

#[derive(Debug, Pod, Zeroable, Clone, Copy)]
#[repr(C)]
pub struct HardwareBuffer {
    data: [u8; 1024],
    size: usize,
}

impl HardwareBuffer {
    // Create a new buffer with capacity for a specific type
    pub fn new() -> Self {
        HardwareBuffer {
            data: [0u8; 1024],
            size: 0,
        }
    }

    /// Write a struct into the buffer
    pub fn write<T: Pod>(&mut self, val: &T) {
        let t_size = size_of::<T>();
        assert!(
            t_size <= self.data.len(),
            "Type too large for HardwareBuffer ({} > {})",
            t_size,
            self.data.len()
        );

        let bytes = bytes_of(val);
        self.data[..t_size].copy_from_slice(bytes);
        self.size = t_size;
    }

    /// Write a raw byte slice into the buffer
    pub fn write_bytes(&mut self, data: &[u8]) {
        assert!(
            data.len() <= self.data.len(),
            "Buffer overflow: tried to write {} bytes into {}-byte buffer",
            data.len(),
            self.data.len()
        );
        self.data[..data.len()].copy_from_slice(data);
        self.size = data.len();
    }

    /// Read a struct back out of the buffer
    pub fn read<T: Pod>(&self) -> Option<T> {
        let t_size = size_of::<T>();
        if self.size != t_size {
            return None;
        }

        Some(*from_bytes(&self.data[..t_size]))
    }

    /// Return the used slice of the buffer
    pub fn as_bytes(&self) -> &[u8] {
        &self.data[..self.size]
    }

    /// Return a mutable slice to the used portion
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        &mut self.data[..self.size]
    }

    /// Return the raw pointer and length (e.g., for FFI)
    pub fn as_c_interface(&self) -> CInterface {
        CInterface {
            data_ptr: self.data.as_ptr(),
            data_len: self.size,
        }
    }

    pub fn as_c_interface_mut(&mut self) -> CInterface {
        CInterface {
            data_ptr: self.data.as_mut_ptr(),
            data_len: self.size,
        }
    }
}
