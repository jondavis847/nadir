use super::{    
    body::{BodyErrors, BodyTrait},
    joint::JointTrait,
    MultibodyTrait,
};
use aerospace::{celestial_system::{CelestialErrors, CelestialSystem}, gravity::Gravity};
use serde::{Deserialize, Serialize};
use spice::Spice;
use uuid::Uuid;

#[derive(Debug)]
pub enum BaseErrors {
    BaseIsCelestial,
    CelestialError(CelestialErrors),
}
impl From<CelestialErrors> for BaseErrors {
    fn from(value: CelestialErrors) -> Self {
        BaseErrors::CelestialError(value)
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum BaseSystems {
    Basic(Option<Gravity>),
    Celestial(CelestialSystem),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Base {
    id: Uuid,
    name: String,
    pub system: BaseSystems,
    pub outer_joints: Vec<Uuid>,
}

impl Default for Base {
    fn default() -> Self {
        Self {
            id: Uuid::new_v4(),
            name: "base".to_string(),
            system: BaseSystems::Basic(None),
            outer_joints: Vec::new(),
        }
    }
}

impl Base {
    pub fn add_celestial_system(&mut self, celestial: CelestialSystem) {
        self.system = BaseSystems::Celestial(celestial);
    }

    pub fn delete_celestial_system(&mut self) {
        self.system = BaseSystems::Basic(None);
    }

    pub fn add_basic_gravity(&mut self, gravity: Gravity) -> Result<(), BaseErrors> {
        match self.system {
            BaseSystems::Basic(_) => self.system = BaseSystems::Basic(Some(gravity)),
            BaseSystems::Celestial(_) => return Err(BaseErrors::BaseIsCelestial),
        }
        Ok(())
    }

    pub fn update(&mut self, t: f64, spice: &mut Option<Spice>) -> Result<(), Box<dyn std::error::Error>> {
        match &mut self.system {
            BaseSystems::Celestial(celestial) => if let Some(spice) = spice {                
                celestial.update(t, spice)?
            } else {
                return Err(CelestialErrors::SpiceNotFound.into())
            },
            BaseSystems::Basic(_) => {}
        }
        Ok(())
    }
}

impl BodyTrait for Base {    
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
