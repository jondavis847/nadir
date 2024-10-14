use super::{
    aerospace::MultibodyGravity,
    body::{BodyErrors, BodyTrait},
    joint::JointTrait,
    MultibodyTrait,
};
use aerospace::celestial_system::{CelestialErrors, CelestialSystem};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

#[derive(Debug)]
pub enum BaseErrors {
    CelestialError(CelestialErrors)
}
impl From<CelestialErrors> for BaseErrors {
    fn from(value: CelestialErrors) -> Self {
        BaseErrors::CelestialError(value)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Base {
    id: Uuid,
    name: String,
    pub celestial: Option<CelestialSystem>,    
    pub outer_joints: Vec<Uuid>,
    pub gravity: Vec<Uuid>,
}

impl Base {
    pub fn new(name: &str) -> Self {
        let mut name = name;
        if name.is_empty() {
            name = "base";
        }

        Self {
            id: Uuid::new_v4(),
            name: name.to_string(),
            celestial: None,
            outer_joints: Vec::new(),
            gravity: Vec::new(),
        }
    }

    pub fn connect_celestial_system(&mut self, celestial: CelestialSystem) {
        self.celestial = Some(celestial);
    }

    pub fn delete_celestial_system(&mut self) {
        self.celestial = None;
    }

    pub fn update(&mut self, t: f64) -> Result<(),BaseErrors> {
        if let Some(celestial) = &mut self.celestial {            
            celestial.update(t)?;
        }
        Ok(())
    }
}

impl BodyTrait for Base {
    fn connect_gravity(&mut self, gravity: &MultibodyGravity) {
        self.gravity.push(*gravity.get_id())
    }
    fn connect_outer_joint<T: JointTrait>(&mut self, joint: &T) -> Result<(), BodyErrors> {
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
