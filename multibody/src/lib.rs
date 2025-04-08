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
use bytemuck::Pod;
use celestial::CelestialErrors;

use joint::JointErrors;
use sensor::SensorErrors;
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

#[derive(Debug)]
pub struct HardwareBuffer(Vec<u8>);

impl HardwareBuffer {
    // Create a new buffer with capacity for a specific type
    pub fn new<T: Pod>() -> Self {
        let size = std::mem::size_of::<T>();
        HardwareBuffer(Vec::with_capacity(size))
    }

    // Write a struct into the buffer
    pub fn write<T: Pod>(&mut self, data: &T) {
        let size = std::mem::size_of::<T>();

        // Ensure the buffer has enough capacity
        if self.0.capacity() < size {
            self.0.reserve(size - self.0.capacity());
        }

        // Reset the buffer
        self.0.clear();

        // Convert the struct to bytes and extend the buffer
        let bytes = bytemuck::bytes_of(data);
        self.0.extend_from_slice(bytes);
    }

    // Read a struct from the buffer
    pub fn read<T: Pod>(&self) -> Option<T> {
        let size = std::mem::size_of::<T>();

        if self.0.len() != size {
            return None; // Buffer doesn't contain exactly one T
        }

        // Convert bytes back to the struct
        Some(*bytemuck::from_bytes::<T>(&self.0))
    }

    // Get a reference to the underlying bytes
    pub fn as_bytes(&self) -> &[u8] {
        &self.0
    }

    // Get a mutable reference to the underlying bytes
    pub fn as_bytes_mut(&mut self) -> &mut [u8] {
        &mut self.0
    }
}
