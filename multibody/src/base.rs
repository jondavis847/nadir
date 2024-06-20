use super::{
    body::{body_enum::BodyEnum, BodyErrors, BodyTrait},
    MultibodyTrait,
};
use mass_properties::MassProperties;
use spatial_algebra::Force;
use uuid::Uuid;

#[derive(Clone, Debug)]
pub struct Base {
    name: String,
    outer_joints: Vec<Uuid>,
    external_force: Force,
    mass_properties: MassProperties,
}

impl Base {
    pub fn new(name: &str) -> Result<BodyEnum, BodyErrors> {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(BodyEnum::Base(Self {
            name: name.to_string(),
            outer_joints: Vec::new(),
            external_force: Force::default(),
            mass_properties: MassProperties::default(),
        }))
    }
}

pub enum BaseErrors {}

impl BodyTrait for Base {
    fn connect_inner_joint(&mut self, _joint_id: &Uuid) -> Result<(), BodyErrors> {
        Err(BodyErrors::NoBaseInnerConnection)
    }

    fn connect_outer_joint(&mut self, joint_id: &Uuid) -> Result<(), BodyErrors> {
        // Check if the joint already exists in outer_joints
        if self.outer_joints.iter().any(|id| id == joint_id) {
            return Err(BodyErrors::OuterJointExists);
        }

        // Push the new joint connection
        self.outer_joints.push(*joint_id);
        Ok(())
    }

    fn delete_inner_joint(&mut self) {
        //nothing to do
    }

    fn delete_outer_joint(&mut self, joint_id: &Uuid) {
        self.outer_joints.retain(|id| id != joint_id);
    }

    fn get_inner_joint(&self) -> &Option<Uuid> {
        &None
    }

    fn get_outer_joints(&self) -> &Vec<Uuid> {
        &self.outer_joints
    }

    fn get_external_force(&self) -> &Force {
        &self.external_force //TODO: this should never get called, can we just make base not be a body?
    }

    fn get_mass_properties(&self) -> &MassProperties {
        &self.mass_properties //TODO: this should never get called, can we just make base not be a body?
    }
}
impl MultibodyTrait for Base {
    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
