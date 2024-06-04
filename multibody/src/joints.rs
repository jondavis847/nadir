use super::MultibodyTrait;
use uuid::Uuid;

pub mod revolute;
use crate::ui::dummies::DummyComponent;
use revolute::Revolute;

#[derive(Debug, Clone)]
pub enum Joint {
    //Floating,
    //Prismatic,
    Revolute(Revolute),
    //Spherical,
}

impl MultibodyTrait for Joint {
    fn connect_from(&mut self, from_id: Uuid) {
        match self {
            Joint::Revolute(joint) => joint.connect_from(from_id),
        }
    }
    fn connect_to(&mut self, to_id: Uuid) {
        match self {
            Joint::Revolute(joint) => joint.connect_to(to_id),
        }
    }

    fn delete_from(&mut self) {
        match self {
            Joint::Revolute(joint) => joint.delete_from(),
        }
    }

    fn delete_to(&mut self, id: Uuid) {
        match self {
            Joint::Revolute(joint) => joint.delete_to(id),
        }
    }

    fn get_component_id(&self) -> Uuid {
        match self {
            Joint::Revolute(revolute) => revolute.get_component_id(),
        }
    }

    fn get_dummy_id(&self) -> Uuid {
        match self {
            Joint::Revolute(revolute) => revolute.get_dummy_id(),
        }
    }

    fn get_from_id(&self) -> Option<Uuid> {
        match self {
            Joint::Revolute(revolute) => revolute.get_from_id(),
        }
    }

    fn get_name(&self) -> &str {
        match self {
            Joint::Revolute(revolute) => revolute.get_name(),
        }
    }

    fn get_node_id(&self) -> Uuid {
        match self {
            Joint::Revolute(revolute) => revolute.get_node_id(),
        }
    }

    fn get_to_id(&self) -> &Vec<Uuid> {
        match self {
            Joint::Revolute(revolute) => revolute.get_to_id(),
        }
    }

    fn inherit_from(&mut self, dummy: &DummyComponent) {
        match self {
            Joint::Revolute(joint) => joint.inherit_from(dummy),
        }
    }
    fn set_component_id(&mut self, id: Uuid) {
        match self {
            Joint::Revolute(revolute) => revolute.set_component_id(id),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            Joint::Revolute(revolute) => revolute.set_name(name),
        }
    }

    fn set_node_id(&mut self, id: Uuid) {
        match self {
            Joint::Revolute(revolute) => revolute.set_node_id(id),
        }
    }

    fn set_system_id(&mut self, id: usize) {
        match self {
            Joint::Revolute(revolute) => revolute.set_system_id(id),
        }
    }
}

#[derive(Debug, Default, Clone, Copy)]
pub struct JointParameters {
    pub constant_force: f64,
    pub dampening: f64,
    pub spring_constant: f64,
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
