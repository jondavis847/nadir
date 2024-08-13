pub mod aerospace;
pub mod algorithms;
pub mod base;
pub mod body;
pub mod component;
pub mod joint;
pub mod result;
pub mod system;
pub mod system_sim;

use uuid::Uuid;
use body::BodyErrors;
use joint::{errors::JointErrors, revolute::RevoluteErrors};

#[derive(Debug,Clone)]
pub enum MultibodyErrors {    
    BaseAlreadyExists,
    BaseMissingOuterJoint,
    BodyNotFound,
    Body(BodyErrors),
    BodyMissingInnerJoint(Uuid),
    ComponentNotFound(String),
    InvalidConnection,
    JointErrors(JointErrors),
    JointMissingInnerBody(Uuid),
    JointMissingOuterBody(Uuid),
    JointNotFound,
    NameTaken,
    NoBaseFound,
    NoTransformFound,
    Revolute(RevoluteErrors),
    TooManyBasesFound,
}

impl From<JointErrors> for MultibodyErrors {
    fn from(e: JointErrors) -> Self {
        MultibodyErrors::JointErrors(e)
    }
}
pub trait MultibodyTrait {
    fn get_id(&self) -> &Uuid;
    fn get_name(&self) -> &str;
    fn set_name(&mut self, name: String);
}