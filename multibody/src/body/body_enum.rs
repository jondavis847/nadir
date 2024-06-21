use super::{Body, BodyErrors, BodyTrait};
use crate::joint::JointEnum;
use crate::{base::Base, MultibodyTrait};
use uuid::Uuid;

#[derive(Clone, Debug)]
pub enum BodyEnum {
    Base(Base),
    Body(Body),
}

impl BodyTrait for BodyEnum {
    fn connect_outer_joint(&mut self, joint: &JointEnum) -> Result<(), BodyErrors> {
        match self {
            BodyEnum::Base(base) => base.connect_outer_joint(joint),
            BodyEnum::Body(body) => body.connect_outer_joint(joint),
        }
    }

    fn delete_outer_joint(&mut self, joint_id: &Uuid) {
        match self {
            BodyEnum::Base(base) => base.delete_outer_joint(joint_id),
            BodyEnum::Body(body) => body.delete_outer_joint(joint_id),
        }
    }

    fn get_outer_joints(&self) -> &Vec<Uuid> {
        match self {
            BodyEnum::Base(base) => base.get_outer_joints(),
            BodyEnum::Body(body) => body.get_outer_joints(),
        }
    }
}

impl MultibodyTrait for BodyEnum {
    fn get_id(&self) -> &Uuid {
        match self {
            BodyEnum::Base(base) => base.get_id(),
            BodyEnum::Body(body) => body.get_id(),
        }
    }

    fn get_name(&self) -> &str {
        match self {
            BodyEnum::Base(base) => base.get_name(),
            BodyEnum::Body(body) => body.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            BodyEnum::Base(base) => base.set_name(name),
            BodyEnum::Body(body) => body.set_name(name),
        }
    }
}
