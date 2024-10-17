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

use base::BaseErrors;
use sensor::SensorErrors;
use spice::SpiceErrors;
use uuid::Uuid;
use body::BodyErrors;
use joint::{errors::JointErrors, revolute::RevoluteErrors};

#[derive(Debug)]
pub enum MultibodyErrors {    
    BaseErrors(BaseErrors),  
    BaseMissingOuterJoint,
    BodyNotFound,
    Body(BodyErrors),
    BodyMissingInnerJoint(Uuid),
    CantDeleteBase,
    ComponentNotFound(String),
    DtCantBeZero,
    InvalidConnection,
    JointErrors(JointErrors),
    JointMissingInnerBody(Uuid),
    JointMissingOuterBody(Uuid),
    JointNotFound,
    NameTaken,
    NoBaseFound,
    NoTransformFound,
    Revolute(RevoluteErrors),
    SensorErrors(SensorErrors),
    SpiceErrors(SpiceErrors),
    TooManyBasesFound,
}

impl From<BaseErrors> for MultibodyErrors {
    fn from(e: BaseErrors) -> Self {
        MultibodyErrors::BaseErrors(e)
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