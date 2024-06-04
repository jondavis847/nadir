use uuid::Uuid;
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
    fn connect_inner(&mut self, id: Uuid) {
        self.meta.id_inner = Some(id);
    }

    fn connect_outer(&mut self, id: Uuid) {
        self.meta.id_outer.push(id);
    }
    fn delete_inner(&mut self) {
        self.meta.id_inner = None;
    }
    fn delete_outer(&mut self, id: Uuid) {
        self.meta.id_outer.retain(|&outer_id| outer_id != id);
    }

    fn get_id(&self) -> Uuid {
        self.meta.id
    }

    fn get_inner_id(&self) -> Option<Uuid> {
        self.meta.id_inner
    }

    fn get_name(&self) -> &str {
        &self.meta.name
    }

    fn get_outer_id(&self) -> &Vec<Uuid> {
        &self.meta.id_outer
    }

    fn set_name(&mut self, name: String) {
        self.meta.name = name;
    }
}
