use std::fmt;
use crate::{joint::JointRef, MultibodyTrait};

// this is just needed for printing so that we don't recursively print the references
#[derive(Clone)]
pub struct BodyJointConnection {
    pub joint: JointRef,
}

impl BodyJointConnection {
    pub fn new(joint: JointRef) -> Self {
        Self { joint }
    }
}

impl fmt::Debug for BodyJointConnection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let joint = self.joint.borrow();

        f.debug_struct("BodyJointConnection")
            .field("joint_name", &joint.get_name())
            .finish()
    }
}
