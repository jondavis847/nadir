use crate::{
    body::BodyRef,
    joint::{Joint, JointConnection, JointErrors, JointParameters, JointRef, JointTrait},
    MultibodyTrait,
};
use std::cell::RefCell;
use std::rc::Rc;

pub enum RevoluteErrors {}

#[derive(Debug, Default, Clone, Copy)]
pub struct RevoluteState {
    pub theta: f64,
    pub omega: f64,
}

impl RevoluteState {
    pub fn new(theta: f64, omega: f64) -> Self {
        Self { theta, omega }
    }
}

#[derive(Debug, Clone)]
pub struct Revolute {
    connection: JointConnection,
    name: String,
    parameters: JointParameters,
    state: RevoluteState,
}

impl Revolute {
    pub fn new(name: &str, parameters: JointParameters, state: RevoluteState) -> JointRef {
        Rc::new(RefCell::new(Joint::Revolute(Self {
            connection: JointConnection::default(),
            name: name.to_string(),
            parameters: parameters,
            state: state,
        })))
    }
}

impl JointTrait for Revolute {
    fn connect_inner_body(&mut self, body: BodyRef) -> Result<(), JointErrors> {
        if self.connection.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists);
        }
        self.connection.inner_body = Some(body);
        Ok(())
    }

    fn connect_outer_body(&mut self, body: BodyRef) -> Result<(), JointErrors> {
        if self.connection.outer_body.is_some() {
            return Err(JointErrors::InnerBodyExists);
        }
        self.connection.outer_body = Some(body);
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

    fn get_inner_body(&self) -> Option<BodyRef> {
        self.connection.inner_body.clone()
    }
    fn get_outer_body(&self) -> Option<BodyRef> {
        self.connection.outer_body.clone()
    }
}

impl MultibodyTrait for Revolute {
    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
