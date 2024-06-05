use sim_value::SimValue;
use std::collections::HashMap;
use uuid::Uuid;

pub mod base;
pub mod body;
pub mod connection;
pub mod joint;
pub mod mass_properties;

use base::{Base, BaseErrors};
use body::{Bodies, Body, BodyErrors, BodyTrait};
use connection::{Connection, ConnectionErrors, Port};
use joint::{revolute::RevoluteErrors, Joint, JointTrait};
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
    bodies: HashMap<Uuid, Bodies<T>>, // MultibodyComponent since its both base and bodies
    connections: HashMap<Uuid, Connection<T>>,
    joints: HashMap<Uuid, Joint<T>>,
}

impl<T> MultibodySystem<T>
where
    T: SimValue,
{
    pub fn new() -> Self {
        Self {
            bodies: HashMap::new(),
            connections: HashMap::new(),
            joints: HashMap::new(),
        }
    }

    pub fn add_base(&mut self, base: Base<T>) {
        self.bodies.insert(base.get_id(), Bodies::Base(base));
    }

    pub fn add_body(&mut self, body: Body<T>) {
        self.bodies.insert(body.get_id(), Bodies::Body(body));
    }

    pub fn add_joint(&mut self, joint: Joint<T>) {
        self.joints.insert(joint.get_id(), joint);
    }

    pub fn connect(
        &mut self,
        from_name: &str,
        to_name: &str,
        transform: Transform<T>,
    ) -> Result<(), ConnectionErrors> {
        let (from_type, from_id) = match self.find_body_id_by_name(from_name) {
            Some(id) => (Port::Body, id),
            None => match self.find_joint_id_by_name(from_name) {
                Some(id) => (Port::Joint, id),
                None => return Err(ConnectionErrors::ComponentNotFound(from_name.to_string())),
            },
        };

        let (to_type, to_id) = match self.find_body_id_by_name(to_name) {
            Some(id) => (Port::Body, id),
            None => match self.find_joint_id_by_name(to_name) {
                Some(id) => (Port::Joint, id),
                None => return Err(ConnectionErrors::ComponentNotFound(to_name.to_string())),
            },
        };

        // ensure connection is valid
        // connect if valid, otherwise return an error
        match (from_type, to_type) {
            (Port::Body, Port::Body) => return Err(ConnectionErrors::BodyToBody),
            (Port::Joint, Port::Joint) => return Err(ConnectionErrors::JointToJoint),
            (Port::Body, Port::Joint) => {
                let body = self.bodies.get_mut(&from_id).unwrap();
                body.connect_outer_joint(to_id, transform).expect("error1");

                let joint = self.joints.get_mut(&to_id).unwrap();
                joint.connect_inner_body(from_id).expect("error2");
            }
            (Port::Joint, Port::Body) => {
                let joint = self.joints.get_mut(&from_id).unwrap();
                joint.connect_outer_body(to_id).expect("error3");

                let body = self.bodies.get_mut(&to_id).unwrap();
                body.connect_inner_joint(from_id, transform)
                    .expect("error4");
            }
            _ => {}
        }
        Ok(())
    }

    fn find_body_id_by_name(&self, name: &str) -> Option<Uuid> {
        let body = self.bodies.values().find(|&body| body.get_name() == name);
        match body {
            None => None,
            Some(body) => Some(body.get_id()),
        }
    }

    fn find_joint_id_by_name(&self, name: &str) -> Option<Uuid> {
        let joint = self.joints.values().find(|&joint| joint.get_name() == name);
        match joint {
            None => None,
            Some(joint) => Some(joint.get_id()),
        }
    }
}
