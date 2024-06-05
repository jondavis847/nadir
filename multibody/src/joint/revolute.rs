use crate::{
    joint::{Joint, JointParameters, JointRef},
    MultibodyTrait,
};
use sim_value::SimValue;
use std::cell::RefCell;
use std::rc::Rc;

pub enum RevoluteErrors {}

#[derive(Debug, Default, Clone, Copy)]
pub struct RevoluteState<T>
where
    T: SimValue,
{
    pub theta: T,
    pub omega: T,
}

impl<T> RevoluteState<T>
where
    T: SimValue,
{
    pub fn new(theta: T, omega: T) -> Self {
        Self { theta, omega }
    }
}

#[derive(Debug, Clone)]
pub struct Revolute<T>
where
    T: SimValue,
{
    name: String,
    parameters: JointParameters<T>,
    state: RevoluteState<T>,
}

impl<T> Revolute<T>
where
    T: SimValue,
{
    pub fn new(name: &str, parameters: JointParameters<T>, state: RevoluteState<T>) -> JointRef<T> {
        Rc::new(RefCell::new(Joint::Revolute(Self {
            name: name.to_string(),
            parameters: parameters,
            state: state,
        })))
    }
}

impl<T> MultibodyTrait for Revolute<T>
where
    T: SimValue,
{
    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
