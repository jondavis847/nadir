use crate::{
    connection::ConnectionErrors,
    joint::{Joint, JointParameters, JointRef, JointTrait},
    MultibodyMeta, MultibodyTrait,
};
use sim_value::SimValue;
use std::cell::RefCell;
use std::rc::Rc;
use uuid::Uuid;

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
    meta: MultibodyMeta,
    parameters: JointParameters<T>,
    state: RevoluteState<T>,
    inner_body: Option<Uuid>,
    outer_body: Option<Uuid>,
}

impl<T> Revolute<T>
where
    T: SimValue,
{
    pub fn new(name: &str, parameters: JointParameters<T>, state: RevoluteState<T>) -> JointRef<T> {
        Rc::new(RefCell::new(Joint::Revolute(Self {
            meta: MultibodyMeta::new(name),
            parameters: parameters,
            state: state,
            inner_body: None,
            outer_body: None,
        })))
    }
}

impl<T> JointTrait for Revolute<T>
where
    T: SimValue,
{
    fn connect_inner_body(&mut self, id: Uuid) -> Result<(), ConnectionErrors> {
        match self.inner_body {
            Some(_) => return Err(ConnectionErrors::JointInnerAlreadyExists),
            None => self.inner_body = Some(id),
        }
        Ok(())
    }
    fn connect_outer_body(&mut self, id: Uuid) -> Result<(), ConnectionErrors> {
        match self.outer_body {
            Some(_) => return Err(ConnectionErrors::JointOuterAlreadyExists),
            None => self.inner_body = Some(id),
        }
        Ok(())
    }
    fn delete_inner_body(&mut self) {
        self.inner_body = None;
    }
    fn delete_outer_body(&mut self) {
        self.outer_body = None;
    }
}

impl<T> MultibodyTrait for Revolute<T>
where
    T: SimValue,
{
    fn get_id(&self) -> Uuid {
        self.meta.id
    }

    fn get_name(&self) -> &str {
        &self.meta.name
    }

    fn set_name(&mut self, name: String) {
        self.meta.name = name;
    }
}
