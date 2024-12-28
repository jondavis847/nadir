pub mod actuator;
pub mod algorithms;
pub mod base;
pub mod body;
pub mod joint;
pub mod sensor;
pub mod software;
pub mod solver;
pub mod system;

use aerospace::celestial_system::CelestialErrors;
use base::BaseErrors;
use body::BodyErrors;
use joint::{revolute::RevoluteErrors, JointErrors};
use sensor::SensorErrors;
use spice::SpiceErrors;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum MultibodyErrors {
    #[error("base error:")]
    BaseErrors(#[from] BaseErrors),
    #[error("base does not have any outer joints")]
    BaseMissingOuterJoint,
    #[error("could not find body '{0}' in the system")]
    BodyNotFound(String),
    #[error("BodyError: {0}")]
    Body(#[from] BodyErrors),
    #[error("body '{0}' does not have an inner joint")]
    BodyMissingInnerJoint(String),
    #[error("base cannot be deleted")]
    CantDeleteBase,
    #[error("celestial error:")]
    CelestialErrors(#[from] CelestialErrors),
    #[error("could not find component {0} in the system")]
    ComponentNotFound(String),
    #[error("could not find state '{0}' for component")]
    ComponentStateNotFound(String),
    #[error("sim dt cannot be 0.0")]
    DtCantBeZero,
    #[error("invalid connection")]
    InvalidConnection,
    #[error("joint error")]
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
    #[error("revolute error: {0}")]
    Revolute(#[from] RevoluteErrors),
    #[error("sensor error: {0}")]
    SensorErrors(#[from] SensorErrors),
    #[error("spice error: {0}")]
    SpiceErrors(#[from] SpiceErrors),
}
