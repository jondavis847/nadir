pub mod actuator;
pub mod algorithms;
pub mod base;
pub mod body;
pub mod component;
pub mod joint;
pub mod result;
pub mod sensor;
pub mod solver;
pub mod system;
pub mod system_sim;

use aerospace::celestial_system::CelestialErrors;
use base::BaseErrors;
use sensor::SensorErrors;
use spice::SpiceErrors;
use uuid::Uuid;
use body::BodyErrors;
use joint::{errors::JointErrors, revolute::RevoluteErrors};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum MultibodyErrors {    
    #[error("base error:")]
    BaseErrors(BaseErrors),  
    #[error("base does not have any outer joints")]
    BaseMissingOuterJoint,
    #[error("could not find body in the system")]
    BodyNotFound,
    #[error("body error:")]
    Body(BodyErrors),
    #[error("body does not have an inner joint")]
    BodyMissingInnerJoint(Uuid),
    #[error("base cannot be deleted")]
    CantDeleteBase,
    #[error("celestial error:")]
    CelestialErrors(CelestialErrors),
    #[error("could not find component in the system")]
    ComponentNotFound(String),
    #[error("could not find state for component")]
    ComponentStateNotFound(String),
    #[error("sim dt cannot be 0.0")]
    DtCantBeZero,
    #[error("invalid connection")]
    InvalidConnection,
    #[error("joint error")]
    JointErrors(JointErrors),
    #[error("joint must have an inner body")]
    JointMissingInnerBody(Uuid),
    #[error("joint must have an outer body")]
    JointMissingOuterBody(Uuid),
    #[error("could not find joint in system")]
    JointNotFound,
    #[error("that name is already taken")]
    NameTaken,    
    #[error("could not find transform")]
    NoTransformFound,
    #[error("revolute error:")]
    Revolute(RevoluteErrors),
    #[error("sensor error:")]
    SensorErrors(SensorErrors),
    #[error("spice error:")]
    SpiceErrors(SpiceErrors),    
}

impl From<BaseErrors> for MultibodyErrors {
    fn from(e: BaseErrors) -> Self {
        MultibodyErrors::BaseErrors(e)
    }
}

impl From<CelestialErrors> for MultibodyErrors {
    fn from(e: CelestialErrors) -> Self {
        MultibodyErrors::CelestialErrors(e)
    }
}


impl From<SpiceErrors> for MultibodyErrors {
    fn from(e: SpiceErrors) -> Self {
        MultibodyErrors::SpiceErrors(e)
    }
}


impl From<JointErrors> for MultibodyErrors {
    fn from(e: JointErrors) -> Self {
        MultibodyErrors::JointErrors(e)
    }
}

impl From<SensorErrors> for MultibodyErrors {
    fn from(e: SensorErrors) -> Self {
        MultibodyErrors::SensorErrors(e)
    }
}
pub trait MultibodyTrait {
    fn get_id(&self) -> &Uuid;
    fn get_name(&self) -> &str;    
    fn set_name(&mut self, name: String);
}