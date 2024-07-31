use super::{Body, BodyErrors, BodyTrait};
use crate::joint::JointTrait;
use crate::{aerospace::MultibodyGravity, base::Base, MultibodyTrait};
use uuid::Uuid;

#[derive(Clone, Debug)]
pub enum BaseOrBody {
    Base(Base),
    Body(Body),
}

impl BodyTrait for BaseOrBody {
    fn connect_gravity(&mut self, gravity: &MultibodyGravity) {
        match self {
            BaseOrBody::Base(base) => base.connect_gravity(gravity),
            BaseOrBody::Body(body) => body.connect_gravity(gravity),
        }
    }

    fn connect_outer_joint<T: JointTrait>(&mut self, joint: &T) -> Result<(), BodyErrors> {
        match self {
            BaseOrBody::Base(base) => base.connect_outer_joint(joint),
            BaseOrBody::Body(body) => body.connect_outer_joint(joint),
        }
    }

    fn delete_outer_joint(&mut self, joint_id: &Uuid) {
        match self {
            BaseOrBody::Base(base) => base.delete_outer_joint(joint_id),
            BaseOrBody::Body(body) => body.delete_outer_joint(joint_id),
        }
    }

    fn get_outer_joints(&self) -> &Vec<Uuid> {
        match self {
            BaseOrBody::Base(base) => base.get_outer_joints(),
            BaseOrBody::Body(body) => body.get_outer_joints(),
        }
    }
}

impl MultibodyTrait for BaseOrBody {
    fn get_id(&self) -> &Uuid {
        match self {
            BaseOrBody::Base(base) => base.get_id(),
            BaseOrBody::Body(body) => body.get_id(),
        }
    }

    fn get_name(&self) -> &str {
        match self {
            BaseOrBody::Base(base) => base.get_name(),
            BaseOrBody::Body(body) => body.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            BaseOrBody::Base(base) => base.set_name(name),
            BaseOrBody::Body(body) => body.set_name(name),
        }
    }
}
