use crate::{joints::JointParameters, MultibodyMeta, MultibodyTrait};
use sim_value::SimValue;

pub enum RevoluteErrors {}

#[derive(Debug, Clone, Copy)]
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
}

impl<T> Revolute<T>
where
    T: SimValue,
{
    pub fn new(
        meta: MultibodyMeta,
        parameters: JointParameters<T>,
        state: RevoluteState<T>,
    ) -> Self {
        Self {
            meta,
            parameters,
            state,
        }
    }
}

impl<T> MultibodyTrait for Revolute<T>
where
    T: SimValue,
{
    fn connect_inner(&mut self, id: usize) {
        self.meta.id.inner = Some(id);
    }

    fn connect_outer(&mut self, id: usize) {
        self.meta.id.outer.push(id);
    }
    fn delete_inner(&mut self) {
        self.meta.id.inner = None;
    }
    fn delete_outer(&mut self, id: usize) {
        self.meta.id.outer.retain(|&outer_id| outer_id != id);
    }

    fn get_id(&self) -> Option<usize> {
        self.meta.id.component
    }

    fn get_inner_id(&self) -> Option<usize> {
        self.meta.id.inner
    }

    fn get_name(&self) -> &str {
        &self.meta.name
    }

    fn get_outer_id(&self) -> &Vec<usize> {
        &self.meta.id.outer
    }

    fn set_id(&mut self, id: usize) {
        self.meta.id.component = Some(id);
    }

    fn set_name(&mut self, name: String) {
        self.meta.name = name;
    }
}
