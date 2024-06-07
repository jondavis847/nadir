use super::{body::BodyRef, MultibodyTrait};
use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;

pub mod revolute;
use revolute::Revolute;

pub type JointRef = Rc<RefCell<JointEnum>>;
pub trait JointTrait {
    fn connect_inner_body(&mut self, body: BodyRef) -> Result<(), JointErrors>;
    fn connect_outer_body(&mut self, body: BodyRef) -> Result<(), JointErrors>;
    fn calculate_transform(&mut self);
    fn delete_inner_body(&mut self);
    fn delete_outer_body(&mut self);
    fn get_inner_body(&self) -> Option<BodyRef>;
    fn get_outer_body(&self) -> Option<BodyRef>;
}

impl JointTrait for JointRef {
    fn connect_inner_body(&mut self, bodyref: BodyRef) -> Result<(), JointErrors> {
        self.borrow_mut().connect_inner_body(bodyref)
    }

    fn connect_outer_body(&mut self, bodyref: BodyRef) -> Result<(), JointErrors> {
        self.borrow_mut().connect_outer_body(bodyref)
    }

    fn calculate_transform(&mut self) {
        self.borrow_mut().calculate_transform();
    }

    fn delete_inner_body(&mut self) {
        self.borrow_mut().delete_inner_body()
    }
    fn delete_outer_body(&mut self) {
        self.borrow_mut().delete_outer_body()
    }

    fn get_inner_body(&self) -> Option<BodyRef> {
        self.borrow().get_inner_body()
    }

    fn get_outer_body(&self) -> Option<BodyRef> {
        self.borrow().get_outer_body()
    }
}

pub enum JointErrors {
    InnerBodyExists,
    OuterBodyExists,
}

#[derive(Debug, Clone)]
pub enum JointEnum {
    //Floating,
    //Prismatic,
    Revolute(Revolute),
    //Spherical,
}

impl MultibodyTrait for JointEnum {
    fn get_name(&self) -> &str {
        match self {
            JointEnum::Revolute(revolute) => revolute.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            JointEnum::Revolute(revolute) => revolute.set_name(name),
        }
    }
}

impl JointTrait for JointEnum {
    fn connect_inner_body(&mut self, body: BodyRef) -> Result<(), JointErrors> {
        match self {
            JointEnum::Revolute(joint) => joint.connect_inner_body(body),
        }
    }
    fn connect_outer_body(&mut self, body: BodyRef) -> Result<(), JointErrors> {
        match self {
            JointEnum::Revolute(joint) => joint.connect_outer_body(body),
        }
    }
    fn calculate_transform(&mut self) {
        match self {
            JointEnum::Revolute(joint) => joint.calculate_transform()
        }
    }
    fn delete_inner_body(&mut self) {
        match self {
            JointEnum::Revolute(joint) => joint.delete_inner_body(),
        }
    }
    fn delete_outer_body(&mut self) {
        match self {
            JointEnum::Revolute(joint) => joint.delete_outer_body(),
        }
    }
    fn get_inner_body(&self) -> Option<BodyRef> {
        match self {
            JointEnum::Revolute(joint) => joint.get_inner_body(),
        }
    }
    fn get_outer_body(&self) -> Option<BodyRef> {
        match self {
            JointEnum::Revolute(joint) => joint.get_outer_body(),
        }
    }
}

#[derive(Default, Clone)]
pub struct JointConnection {
    inner_body: Option<BodyRef>,
    outer_body: Option<BodyRef>,
}

impl fmt::Debug for JointConnection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let inner_body = match &self.inner_body {
            None => None,
            Some(body) => Some(body.borrow().get_name().to_string()),
        };

        let outer_body = match &self.outer_body {
            None => None,
            Some(body) => Some(body.borrow().get_name().to_string()),
        };

        f.debug_struct("JointConnection")
            .field("inner_body", &inner_body)
            .field("outer_body", &outer_body)
            .finish()
    }
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
