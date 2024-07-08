pub mod algorithms;
pub mod base;
pub mod body;
pub mod joint;
pub mod system;
pub mod system_sim;

use uuid::Uuid;
use body::BodyErrors;
use joint::revolute::RevoluteErrors;

#[derive(Debug, Copy,Clone)]
pub enum MultibodyErrors {    
    BaseAlreadyExists,
    BaseMissingOuterJoint,
    Body(BodyErrors),
    BodyMissingInnerJoint(Uuid),
    JointMissingInnerBody(Uuid),
    JointMissingOuterBody(Uuid),
    JointNotFound,
    NameTaken,
    NoBaseFound,
    Revolute(RevoluteErrors),
    TooManyBasesFound,
}

pub trait MultibodyTrait {
    fn get_id(&self) -> &Uuid;
    fn get_name(&self) -> &str;
    fn set_name(&mut self, name: String);
}