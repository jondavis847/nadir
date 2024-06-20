use super::{Body, BodyErrors, BodyTrait};
use crate::{base::Base, MultibodyTrait};
use mass_properties::MassProperties;
use spatial_algebra::Force;
use uuid::Uuid;

#[derive(Clone, Debug)]
pub enum BodyEnum {
    Base(Base),
    Body(Body),
}

impl BodyTrait for BodyEnum {
    fn connect_inner_joint(&mut self, joint_id: &Uuid) -> Result<(), BodyErrors> {
        match self {
            BodyEnum::Base(base) => base.connect_inner_joint(joint_id),
            BodyEnum::Body(body) => body.connect_inner_joint(joint_id),
        }
    }

    fn connect_outer_joint(&mut self, joint_id: &Uuid) -> Result<(), BodyErrors> {
        match self {
            BodyEnum::Base(base) => base.connect_outer_joint(joint_id),
            BodyEnum::Body(body) => body.connect_outer_joint(joint_id),
        }
    }

    fn delete_inner_joint(&mut self) {
        match self {
            BodyEnum::Base(base) => base.delete_inner_joint(),
            BodyEnum::Body(body) => body.delete_inner_joint(),
        }
    }
    fn delete_outer_joint(&mut self, joint_id: &Uuid) {
        match self {
            BodyEnum::Base(base) => base.delete_outer_joint(joint_id),
            BodyEnum::Body(body) => body.delete_outer_joint(joint_id),
        }
    }

    fn get_external_force(&self) -> &Force {
        match self {
            BodyEnum::Base(base) => base.get_external_force(),
            BodyEnum::Body(body) => body.get_external_force(),
        }
    }

    fn get_inner_joint(&self) -> &Option<Uuid> {
        match self {
            BodyEnum::Base(base) => base.get_inner_joint(),
            BodyEnum::Body(body) => body.get_inner_joint(),
        }
    }

    fn get_outer_joints(&self) -> &Vec<Uuid> {
        match self {
            BodyEnum::Base(base) => base.get_outer_joints(),
            BodyEnum::Body(body) => body.get_outer_joints(),
        }
    }

    #[inline]
    fn get_mass_properties(&self) -> &MassProperties {
        match self {
            BodyEnum::Base(base) => base.get_mass_properties(),
            BodyEnum::Body(body) => body.get_mass_properties(),
        }
    }
}

impl MultibodyTrait for BodyEnum {
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
