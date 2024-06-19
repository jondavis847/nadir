use mass_properties::MassProperties;
use spatial_algebra::Force;
use std::cell::RefCell;
use std::rc::Rc;

use super::{body_enum::BodyEnum, connection_joint::BodyJointConnection, BodyErrors, BodyTrait};
use crate::joint::JointRef;

pub type BodyRef = Rc<RefCell<BodyEnum>>;

impl BodyTrait for BodyRef {
    #[inline]
    fn connect_inner_joint(&mut self, jointref: JointRef) -> Result<(), BodyErrors> {
        self.borrow_mut().connect_inner_joint(jointref)
    }

    #[inline]
    fn connect_outer_joint(&mut self, jointref: JointRef) -> Result<(), BodyErrors> {
        self.borrow_mut().connect_outer_joint(jointref)
    }

    #[inline]
    fn delete_inner_joint(&mut self) {
        self.borrow_mut().delete_inner_joint()
    }

    #[inline]
    fn delete_outer_joint(&mut self, jointref: JointRef) {
        self.borrow_mut().delete_outer_joint(jointref)
    }

    #[inline]
    fn get_inner_joint(&self) -> Option<BodyJointConnection> {
        self.borrow().get_inner_joint()
    }

    #[inline]
    fn get_outer_joints(&self) -> Vec<BodyJointConnection> {
        self.borrow().get_outer_joints()
    }

    #[inline]
    fn get_mass_properties(&self) -> MassProperties {
        self.borrow().get_mass_properties()
    }

    #[inline]
    fn get_external_force(&self) -> Force {
        self.borrow().get_external_force()
    }
}
