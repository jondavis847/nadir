use super::{
    body::{BodyRef, BodyTrait},
    MultibodyTrait,
};
use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;
use transforms::Transform;

pub mod revolute;
use revolute::Revolute;

pub type JointRef = Rc<RefCell<JointEnum>>;
pub trait JointTrait {
    fn connect_inner_body(
        &mut self,
        body: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors>;
    fn connect_outer_body(
        &mut self,
        body: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors>;
    fn delete_inner_body(&mut self);
    fn delete_outer_body(&mut self);
    fn get_inner_body(&self) -> Option<Connection>;
    fn get_outer_body(&self) -> Option<Connection>;
    fn get_transforms(&self) -> JointTransforms;
    fn update_transforms(&mut self);
}

impl JointTrait for JointRef {
    fn connect_inner_body(
        &mut self,
        bodyref: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        self.borrow_mut().connect_inner_body(bodyref, transform)
    }

    fn connect_outer_body(
        &mut self,
        bodyref: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        self.borrow_mut().connect_outer_body(bodyref, transform)
    }

    fn delete_inner_body(&mut self) {
        self.borrow_mut().delete_inner_body()
    }
    fn delete_outer_body(&mut self) {
        self.borrow_mut().delete_outer_body()
    }

    fn get_inner_body(&self) -> Option<Connection> {
        self.borrow().get_inner_body()
    }

    fn get_outer_body(&self) -> Option<Connection> {
        self.borrow().get_outer_body()
    }

    fn get_transforms(&self) -> JointTransforms {
        self.borrow().get_transforms()
    }
    fn update_transforms(&mut self) {
        self.borrow_mut().update_transforms()
    }
}

pub enum JointErrors {
    InnerBodyExists,
    OuterBodyExists,
}

#[derive(Debug, Clone)]
pub struct JointCommon {
    pub name: String,
    pub connection: JointConnection,
    pub transforms: JointTransforms,
}

impl JointCommon {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            connection: JointConnection::default(),
            transforms: JointTransforms::default(),
        }
    }
}
impl JointCommon {
    pub fn update_transforms(&mut self) {
        // transforms are multiplied like matrices from right to left.
        // i.e. if you want to express v from frame A in frame C
        // you would use vC = C_to_B * B_to_A * vA
        // this means that the transform outer_body_to_inner_body is actually a
        // transform from the inner body to the outer body
        // I just like this notation better

        // get relevant transforms from the parent for calculations  to the base
        let parent = self.connection.inner_body.as_ref().unwrap().body.borrow();
        let parent_joint_connection = parent.get_inner_joint().unwrap();
        let parent_joint = parent_joint_connection.joint.borrow();
        let parent_joint_transforms = parent_joint.get_transforms();
        let ib_to_base = parent_joint_transforms.outer_body_to_base;

        // get transforms for calculations
        let jif_to_jof = self.transforms.jif_to_jof;
        let jof_to_jif = jif_to_jof.inv();
        let ib_to_jif = self.connection.inner_body.as_ref().unwrap().transform;
        let jif_to_ib = ib_to_jif.inv();
        let ob_to_jof = self.connection.outer_body.as_ref().unwrap().transform;

        // calculate the transforms
        let jof_to_ib = jof_to_jif * jif_to_ib;
        let ib_to_jof = jof_to_ib.inv();
        let jof_to_base = jof_to_ib * ib_to_base;
        let base_to_jof = jof_to_base.inv();
        let ob_to_ib = ob_to_jof * jof_to_ib;
        let ib_to_ob = ob_to_ib.inv();
        let ob_to_base = ob_to_ib * ib_to_base;
        let base_to_ob = ob_to_base.inv();

        self.transforms.jof_to_inner_body = jof_to_ib;
        self.transforms.inner_body_to_jof = ib_to_jof;
        self.transforms.jof_to_base = jof_to_base;
        self.transforms.base_to_jof = base_to_jof;
        self.transforms.outer_body_to_inner_body = ob_to_ib;
        self.transforms.inner_body_to_outer_body = ib_to_ob;
        self.transforms.outer_body_to_base = ob_to_base;
        self.transforms.base_to_outer_body = base_to_ob;
    }
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
    fn connect_inner_body(
        &mut self,
        body: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        match self {
            JointEnum::Revolute(joint) => joint.connect_inner_body(body, transform),
        }
    }
    fn connect_outer_body(
        &mut self,
        body: BodyRef,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        match self {
            JointEnum::Revolute(joint) => joint.connect_outer_body(body, transform),
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
    fn get_inner_body(&self) -> Option<Connection> {
        match self {
            JointEnum::Revolute(joint) => joint.get_inner_body(),
        }
    }
    fn get_outer_body(&self) -> Option<Connection> {
        match self {
            JointEnum::Revolute(joint) => joint.get_outer_body(),
        }
    }

    fn get_transforms(&self) -> JointTransforms {
        match self {
            JointEnum::Revolute(joint) => joint.get_transforms(),
        }
    }

    fn update_transforms(&mut self) {
        match self {
            JointEnum::Revolute(joint) => joint.update_transforms(),
        }
    }
}

// We choose to keep the transform information with the joint rather than the body.
// This is because bodies can have multiple outer joints where as a joint only has one inner or outer body.
// If it was in the body, and I want the the transform from parent body to this body,
// then I need the transform from parent body to this inner joint.
// To get that transform with that info in the parent body, we would need to search it's
// outer joints for this inner joint and apply that transform.
// If the transform is already in the joint, then no searching.
// We also choose to make the transform be from the body frame to the joint frame.
// This is because the location of things like actuators or other bodies are typically expressed in the body frame.

#[derive(Clone)]
pub struct Connection {
    pub body: BodyRef,
    pub transform: Transform,
}
impl Connection {
    pub fn new(body: BodyRef, transform: Transform) -> Self {
        Self { body, transform }
    }
}

#[derive(Default, Clone)]
pub struct JointConnection {
    inner_body: Option<Connection>,
    outer_body: Option<Connection>,
}

// Better printing so we dont recurse with bodyref and jointref
impl fmt::Debug for JointConnection {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let inner_body = match &self.inner_body {
            None => None,
            Some(connection) => Some(connection.body.borrow().get_name().to_string()),
        };

        let outer_body = match &self.outer_body {
            None => None,
            Some(connection) => Some(connection.body.borrow().get_name().to_string()),
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

/// base: the "body/base frame" of the base
/// inner_body: the "body frame" of the body on the base side of the joint
/// outer_body: the "body frame" of the body on the tip side of the joint
/// jif: the "joint inner frame"
/// jof: the "joint outer frame"
#[derive(Clone, Copy, Debug, Default)]
pub struct JointTransforms {
    jif_to_jof: Transform,
    jof_to_inner_body: Transform,
    jof_to_base: Transform,
    inner_body_to_jof: Transform,
    base_to_jof: Transform,
    outer_body_to_inner_body: Transform,
    inner_body_to_outer_body: Transform,
    outer_body_to_base: Transform,
    base_to_outer_body: Transform,
}
