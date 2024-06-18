pub mod algorithms;
pub mod base;
pub mod body;
pub mod component;
pub mod joint;
pub mod system;

use base::BaseErrors;
use body::{BodyErrors, BodyRef};
use joint::{revolute::RevoluteErrors, JointRef};

pub enum MultibodyErrors {
    Base(BaseErrors),
    BaseAlreadyExists,
    BaseMissingOuterJoint,
    Body(BodyErrors),
    BodyMissingInnerJoint(BodyRef),
    JointMissingInnerBody(JointRef),
    JointMissingOuterBody(JointRef),
    NameTaken,
    NoBaseFound,
    Revolute(RevoluteErrors),
    TooManyBasesFound,
}

pub trait MultibodyTrait {
    fn get_name(&self) -> &str;
    fn set_name(&mut self, name: String);
}
