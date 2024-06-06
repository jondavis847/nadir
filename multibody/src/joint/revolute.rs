use crate::{
    body::BodyRef,
    joint::{Joint, JointConnection, JointErrors, JointParameters, JointRef, JointTrait},
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
    connection: JointConnection<T>,
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
            connection: JointConnection::default(),
            name: name.to_string(),
            parameters: parameters,
            state: state,
        })))
    }
}

impl<T> JointTrait<T> for Revolute<T>
where
    T: SimValue,
{
    fn connect_inner_body(&mut self, body: BodyRef<T>) -> Result<(), JointErrors> {
        if self.connection.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists);
        }
        self.connection.inner_body = Some(body);
        Ok(())
    }

    fn connect_outer_body(&mut self, body: BodyRef<T>) -> Result<(), JointErrors> {
        if self.connection.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists);
        }
        self.connection.inner_body = Some(body);
        Ok(())
    }

    fn delete_inner_body(&mut self) {
        if self.connection.inner_body.is_some() {
            self.connection.inner_body = None;
        }
    }

    fn delete_outer_body(&mut self) {
        if self.connection.outer_body.is_some() {
            self.connection.outer_body = None;
        }
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
