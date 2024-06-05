use super::{connection::ConnectionErrors, MultibodyTrait};
use sim_value::SimValue;
use uuid::Uuid;

pub mod revolute;
use revolute::Revolute;

pub trait JointTrait {
    fn connect_inner_body(&mut self, id: Uuid) -> Result<(), ConnectionErrors>;
    fn connect_outer_body(&mut self, id: Uuid) -> Result<(), ConnectionErrors>;
    fn delete_inner_body(&mut self);
    fn delete_outer_body(&mut self);
}

#[derive(Debug, Clone)]
pub enum Joint<T>
where
    T: SimValue,
{
    //Floating,
    //Prismatic,
    Revolute(Revolute<T>),
    //Spherical,
}

impl<T> JointTrait for Joint<T>
where
    T: SimValue,
{
    fn connect_inner_body(&mut self, id: Uuid) -> Result<(), ConnectionErrors> {
        match self {
            Joint::Revolute(joint) => joint.connect_inner_body(id),
        }
    }
    fn connect_outer_body(&mut self, id: Uuid) -> Result<(), ConnectionErrors> {
        match self {
            Joint::Revolute(joint) => joint.connect_outer_body(id),
        }
    }
    fn delete_inner_body(&mut self) {
        match self {
            Joint::Revolute(joint) => joint.delete_inner_body(),
        }
    }
    fn delete_outer_body(&mut self) {
        match self {
            Joint::Revolute(joint) => joint.delete_outer_body(),
        }
    }
}

impl<T> MultibodyTrait for Joint<T>
where
    T: SimValue,
{
    fn get_id(&self) -> Uuid {
        match self {
            Joint::Revolute(revolute) => revolute.get_id(),
        }
    }

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

#[derive(Debug, Default, Clone, Copy)]
pub struct JointParameters<T>
where
    T: SimValue,
{
    constant_force: T,
    dampening: T,
    spring_constant: T,
}

impl<T> JointParameters<T>
where
    T: SimValue,
{
    pub fn new(constant_force: T, dampening: T, spring_constant: T) -> Self {
        Self {
            constant_force,
            dampening,
            spring_constant,
        }
    }
}
