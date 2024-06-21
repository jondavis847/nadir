use super::{
    body::{body_enum::BodyEnum, BodyErrors, BodyTrait},
    joint::{JointEnum, JointTrait},
    MultibodyTrait,
};
use mass_properties::MassProperties;
use spatial_algebra::Force;
use uuid::Uuid;

#[derive(Clone, Debug)]
pub struct Base {
    id: Uuid,
    name: String,
    outer_joints: Vec<Uuid>,
    external_force: Force,
    mass_properties: MassProperties,
}

impl Base {
    pub fn new(name: &str) -> Result<Base, BaseErrors> {
        let mut name = name;
        if name.is_empty() {
            name = "base";
        }
        Ok(Self {
            id: Uuid::new_v4(),
            name: name.to_string(),
            outer_joints: Vec::new(),
            external_force: Force::default(),
            mass_properties: MassProperties::default(),
        })
    }
}

pub enum BaseErrors {}

impl BodyTrait for Base {
    fn connect_outer_joint(&mut self, joint: &JointEnum) -> Result<(), BodyErrors> {
        let joint_id = joint.get_id();
        // Check if the joint already exists in outer_joints
        if self.outer_joints.iter().any(|id| id == joint_id) {
            return Err(BodyErrors::OuterJointExists);
        }

        // Push the new joint connection
        self.outer_joints.push(*joint_id);
        Ok(())
    }

    fn delete_outer_joint(&mut self, joint_id: &Uuid) {
        self.outer_joints.retain(|id| id != joint_id);
    }

    fn get_outer_joints(&self) -> &Vec<Uuid> {
        &self.outer_joints
    }
}
impl MultibodyTrait for Base {
    fn get_id(&self) -> &Uuid {
        &self.id
    }

    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
