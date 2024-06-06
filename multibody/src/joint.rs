use super::{body::BodyRef, MultibodyTrait};
use std::cell::RefCell;
use std::rc::Rc;

pub mod revolute;
use revolute::Revolute;

pub type JointRef = Rc<RefCell<Joint>>;

pub enum JointErrors {
    InnerBodyExists,
    OuterBodyExists,
}

#[derive(Debug, Clone)]
pub enum Joint {
    //Floating,
    //Prismatic,
    Revolute(Revolute),
    //Spherical,
}

impl MultibodyTrait for Joint {
    fn get_name(&self) -> &str {
        match self {
            Joint::Revolute(revolute) => revolute.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            Joint::Revolute(revolute) => revolute.set_name(name),
        }
    }
}

pub trait JointTrait {
    fn connect_inner_body(&mut self, body: BodyRef) -> Result<(), JointErrors>;
    fn connect_outer_body(&mut self, body: BodyRef) -> Result<(), JointErrors>;
    fn delete_inner_body(&mut self);
    fn delete_outer_body(&mut self);
}

#[derive(Debug, Default, Clone)]
pub struct JointConnection {
    inner_body: Option<BodyRef>,
    outer_body: Option<BodyRef>,
}

#[derive(Debug, Default, Clone, Copy)]
pub struct JointParameters {
    constant_force: f64,
    dampening: f64,
    spring_constant: f64,
}

impl JointParameters {
    pub fn new(constant_force: f64, dampening: f64, spring_constant: f64) -> Self {
        Self {
            constant_force,
            dampening,
            spring_constant,
        }
    }
}
