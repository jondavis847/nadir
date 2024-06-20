use super::{base::Base, joint::JointRef, MultibodyTrait};
use mass_properties::{MassProperties, MassPropertiesErrors};
use spatial_algebra::Force;
use uuid::Uuid;

pub mod body_enum;
pub mod body_state;
use body_enum::BodyEnum;
use body_state::BodyState;

#[derive(Clone, Copy, Debug)]
pub enum BodyErrors {
    EmptyName,
    InnerJointExists,
    MassPropertiesErrors(MassPropertiesErrors),
    NoBaseInnerConnection,
    OuterJointExists,
}

pub trait BodyTrait {
    fn connect_inner_joint(&mut self, joint_id: &Uuid) -> Result<(), BodyErrors>;

    fn connect_outer_joint(&mut self, joint_id: &Uuid) -> Result<(), BodyErrors>;

    fn delete_inner_joint(&mut self);
    fn delete_outer_joint(&mut self, joint_id: &Uuid);
    fn get_external_force(&self) -> &Force;
    fn get_inner_joint(&self) -> &Option<Uuid>;
    fn get_outer_joints(&self) -> &Vec<Uuid>;
    fn get_mass_properties(&self) -> &MassProperties;
}

#[derive(Debug, Clone)]
pub struct Body {
    //actuators: Vec<BodyActuatorConnection>,
    inner_joint: Option<Uuid>,
    mass_properties: MassProperties,
    name: String,
    outer_joints: Vec<Uuid>,
    //sensors: Vec<BodySensorConnection>,
    state: BodyState,
}

impl Body {
    pub fn new(name: &str, mass_properties: MassProperties) -> Result<BodyEnum, BodyErrors> {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(BodyEnum::Body(Self {
            //actuators: Vec::new(),
            inner_joint: None,
            mass_properties: mass_properties,
            name: name.to_string(),
            outer_joints: Vec::new(),
            state: BodyState::default(),
        }))
    }
}

impl BodyTrait for Body {
    fn connect_inner_joint(&mut self, joint_id: &Uuid) -> Result<(), BodyErrors> {
        match self.inner_joint {
            Some(_) => return Err(BodyErrors::InnerJointExists),
            None => self.inner_joint = Some(*joint_id),
        }
        Ok(())
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
        if self.inner_joint.is_some() {
            self.inner_joint = None;
        }
    }

    fn delete_outer_joint(&mut self, joint_id: &Uuid) {
        self.outer_joints.retain(|id| id != joint_id);
    }

    fn get_external_force(&self) -> &Force {
        &self.state.external_force
    }

    fn get_inner_joint(&self) -> &Option<Uuid> {
        &self.inner_joint
    }

    fn get_outer_joints(&self) -> &Vec<Uuid> {
        &self.outer_joints
    }

    fn get_mass_properties(&self) -> &MassProperties {
        &self.mass_properties
    }
}

impl MultibodyTrait for Body {
    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
