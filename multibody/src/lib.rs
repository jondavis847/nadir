use sim_value::SimValue;
use std::cell::RefCell;
use std::collections::HashMap;
use std::rc::Rc;
use uuid::Uuid;

pub mod base;
pub mod body;
pub mod connection;
pub mod joint;
pub mod mass_properties;

use base::{Base, BaseErrors};
use body::{Bodies, Body, BodyErrors, BodyRef};
use connection::{Connection, ConnectionErrors, Port};
use joint::{revolute::RevoluteErrors, Joint, JointRef};
use transforms::Transform;

#[derive(Debug, Clone)]
pub struct MultibodyMeta {
    name: String,
    id: Uuid,
}

impl MultibodyMeta {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            id: Uuid::new_v4(),
        }
    }
}

pub enum MultibodyErrors {
    Base(BaseErrors),
    Body(BodyErrors),
    Revolute(RevoluteErrors),
}

pub trait MultibodyTrait {
    fn get_id(&self) -> Uuid;
    fn get_name(&self) -> &str;
    fn set_name(&mut self, name: String);
}

#[derive(Debug, Clone)]
pub enum MultibodyComponent<T>
where
    T: SimValue,
{
    Base(Base<T>),
    Body(Body<T>),
    Joint(Joint<T>),
}

impl<T> MultibodyTrait for MultibodyComponent<T>
where
    T: SimValue,
{
    fn get_id(&self) -> Uuid {
        match self {
            MultibodyComponent::Base(base) => base.get_id(),
            MultibodyComponent::Body(body) => body.get_id(),
            MultibodyComponent::Joint(joint) => joint.get_id(),
        }
    }

    fn get_name(&self) -> &str {
        match self {
            MultibodyComponent::Base(base) => base.get_name(),
            MultibodyComponent::Body(body) => body.get_name(),
            MultibodyComponent::Joint(joint) => joint.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            MultibodyComponent::Base(base) => base.set_name(name),
            MultibodyComponent::Body(body) => body.set_name(name),
            MultibodyComponent::Joint(joint) => joint.set_name(name),
        }
    }
}

#[derive(Debug, Clone)]
pub struct MultibodySystem<T>
where
    T: SimValue,
{
    bodies: Vec<BodyRef<T>>, // MultibodyComponent since its both base and bodies
    connections: Vec<Connection<T>>,
    joints: Vec<JointRef<T>>,
}

impl<T> MultibodySystem<T>
where
    T: SimValue,
{
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            connections: Vec::new(),
            joints: Vec::new(),
        }
    }

    pub fn add_body(&mut self, body: BodyRef<T>) {
        self.bodies.push(body);
    }

    pub fn add_joint(&mut self, joint: JointRef<T>) {
        self.joints.push(joint);
    }

    pub fn connect(
        &mut self,
        joint_name: &str,
        inner_body_name: &str,
        inner_body_frame: Transform<T>,
        outer_body_name: &str,
        outer_body_frame: Transform<T>,
    ) -> Result<(), ConnectionErrors> {
        let joint = match self.find_joint_by_name(joint_name) {
            Some(joint) => joint,
            None => return Err(ConnectionErrors::ComponentNotFound),
        };
        let inner_body = match self.find_body_by_name(inner_body_name) {
            Some(body) => body,
            None => return Err(ConnectionErrors::ComponentNotFound),
        };
        let outer_body = match self.find_body_by_name(outer_body_name) {
            Some(body) => body,
            None => return Err(ConnectionErrors::ComponentNotFound),
        };

        let connection = Connection::new(
            joint,
            inner_body,
            inner_body_frame,
            outer_body,
            outer_body_frame,
        );
        self.connections.push(connection);
        Ok(())
    }

    pub fn find_body_by_name(&self, name: &str) -> Option<BodyRef<T>> {
        self.bodies
            .iter()
            .find(|joint| joint.borrow().get_name() == name)
            .cloned()
    }

    pub fn find_joint_by_name(&self, name: &str) -> Option<JointRef<T>> {
        self.joints
            .iter()
            .find(|joint| joint.borrow().get_name() == name)
            .cloned()
    }
}
