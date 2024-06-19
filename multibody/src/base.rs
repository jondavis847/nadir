use super::{
    body::{
        body_enum::BodyEnum, body_ref::BodyRef, connection_joint::BodyJointConnection, BodyErrors,
        BodyTrait,
    },
    joint::JointRef,
    MultibodyTrait,
};
use mass_properties::MassProperties;
use spatial_algebra::Force;
use std::cell::RefCell;
use std::rc::Rc;

pub type BaseRef = Rc<RefCell<Base>>;
#[derive(Debug, Clone)]
pub struct Base {
    name: String,
    outer_joints: Vec<BodyJointConnection>,
}

impl Base {
    pub fn new(name: &str) -> Result<BodyRef, BodyErrors> {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(Rc::new(RefCell::new(BodyEnum::Base(Self {
            name: name.to_string(),
            outer_joints: Vec::new(),
        }))))
    }
}

pub enum BaseErrors {}

impl BodyTrait for Base {
    fn connect_inner_joint(&mut self, _jointref: JointRef) -> Result<(), BodyErrors> {
        Err(BodyErrors::NoBaseInnerConnection)
    }

    fn connect_outer_joint(&mut self, jointref: JointRef) -> Result<(), BodyErrors> {
        // Borrow the joint and get its name
        let joint_name = jointref.borrow().get_name().to_string();

        // Check if the joint already exists in outer_joints
        if self
            .outer_joints
            .iter()
            .any(|connection| connection.joint.borrow().get_name() == joint_name)
        {
            return Err(BodyErrors::OuterJointExists);
        }

        // Push the new joint connection
        self.outer_joints.push(BodyJointConnection::new(jointref));
        Ok(())
    }

    fn delete_inner_joint(&mut self) {
        //nothing to do
    }

    fn delete_outer_joint(&mut self, jointref: JointRef) {
        self.outer_joints
            .retain(|connection| !Rc::ptr_eq(&connection.joint, &jointref));
    }

    fn get_inner_joint(&self) -> Option<BodyJointConnection> {
        None
    }

    fn get_outer_joints(&self) -> Vec<BodyJointConnection> {
        self.outer_joints.clone()
    }

    fn get_external_force(&self) -> Force {
        Force::default()
    }

    fn get_mass_properties(&self) -> MassProperties {
        MassProperties::default() //TODO: this should never get called, can we just make base not be a body?
    }
}
impl MultibodyTrait for Base {
    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
