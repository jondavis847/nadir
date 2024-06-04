use uuid::Uuid;
use super::MultibodyTrait;
use sim_value::SimValue;

pub mod revolute;
use revolute::Revolute;

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

impl<T> MultibodyTrait for Joint<T>
where
    T: SimValue,
{
    fn connect_inner(&mut self, id: Uuid) {
        match self {
            Joint::Revolute(joint) => joint.connect_inner(id),
        }
    }
    fn connect_outer(&mut self, id: Uuid) {
        match self {
            Joint::Revolute(joint) => joint.connect_outer(id),
        }
    }

    fn delete_inner(&mut self) {
        match self {
            Joint::Revolute(joint) => joint.delete_inner(),
        }
    }

    fn delete_outer(&mut self, id: Uuid) {
        match self {
            Joint::Revolute(joint) => joint.delete_outer(id),
        }
    }

    fn get_id(&self) -> Uuid {
        match self {
            Joint::Revolute(revolute) => revolute.get_id(),
        }
    }

    fn get_inner_id(&self) -> Option<Uuid> {
        match self {
            Joint::Revolute(revolute) => revolute.get_inner_id(),
        }
    }

    fn get_name(&self) -> &str {
        match self {
            Joint::Revolute(revolute) => revolute.get_name(),
        }
    }

    fn get_outer_id(&self) -> &Vec<Uuid> {
        match self {
            Joint::Revolute(revolute) => revolute.get_outer_id(),
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
