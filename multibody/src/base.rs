use super::{
    body::{BodyConnection, BodyTrait},
    MultibodyMeta, MultibodyTrait,
};
use crate::connection::ConnectionErrors;
use sim_value::SimValue;
use transforms::Transform;
use uuid::Uuid;

#[derive(Debug, Clone)]
pub struct Base<T>
where
    T: SimValue,
{
    pub meta: MultibodyMeta,
    outer_joints: Vec<BodyConnection<T>>,
}

impl<T> Base<T>
where
    T: SimValue,
{
    pub fn new(name: &str) -> Self {
        Self {
            meta: MultibodyMeta::new(name),
            outer_joints: Vec::new(),
        }
    }
}

pub enum BaseErrors {}

impl<T> BodyTrait<T> for Base<T>
where
    T: SimValue,
{
    fn connect_inner_joint(
        &mut self,
        _id: Uuid,
        _transform: Transform<T>,
    ) -> Result<(), ConnectionErrors> {
        Err(ConnectionErrors::NothingBeforeBase)
    }

    fn connect_outer_joint(
        &mut self,
        id: Uuid,
        transform: Transform<T>,
    ) -> Result<(), ConnectionErrors> {
        if !self
            .outer_joints
            .iter()
            .any(|connection| connection.get_joint_id() == id)
        {
            let connection = BodyConnection::new(id, transform);
            self.outer_joints.push(connection);
        }
        Ok(())
    }

    fn delete_inner_joint(&mut self) {
        //nothing to delete for base
    }

    fn delete_outer_joint(&mut self, id: Uuid) {
        self.outer_joints
            .retain(|connection| connection.get_joint_id() != id)
    }
}

impl<T> MultibodyTrait for Base<T>
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
