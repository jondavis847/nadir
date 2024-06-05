use super::MultibodyTrait;
use sim_value::SimValue;
use std::cell::RefCell;
use std::rc::Rc;

pub mod revolute;
use revolute::Revolute;

pub type JointRef<T> = Rc<RefCell<Joint<T>>>;

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
