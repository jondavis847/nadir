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
use joint::revolute::RevoluteErrors;

#[derive(Debug,Clone)]
pub enum MultibodyErrors {    
    BaseAlreadyExists,
    BaseMissingOuterJoint,
    BodyNotFound,
    Body(BodyErrors),
    BodyMissingInnerJoint(Uuid),
    ComponentNotFound(String),
    InvalidConnection,
    JointMissingInnerBody(Uuid),
    JointMissingOuterBody(Uuid),
    JointNotFound,
    NameTaken,
    NoBaseFound,
    NoTransformFound,
    Revolute(RevoluteErrors),
    TooManyBasesFound,
}

pub trait MultibodyTrait {
    fn get_id(&self) -> &Uuid;
    fn get_name(&self) -> &str;
    fn set_name(&mut self, name: String);
}