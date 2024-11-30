use crate::joint::Joint;

use super::body::{BodyErrors, BodyTrait};
use aerospace::{
    celestial_system::{CelestialErrors, CelestialSystem},
    gravity::Gravity,
};
use serde::{Deserialize, Serialize};
use spice::Spice;
use thiserror::Error;

#[derive(Debug,Error)]
pub enum BaseErrors {
    #[error("base is celestial")]
    BaseIsCelestial,
    #[error("CelestialError: {0}")]
    CelestialError(#[from]CelestialErrors),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum BaseSystems {
    Basic(Option<Gravity>),
    Celestial(CelestialSystem),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Base {
    pub name: String,
    pub system: BaseSystems,
    pub outer_joints: Vec<String>,
}

impl Default for Base {
    fn default() -> Self {
        Self {
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

    pub fn update(
        &mut self,
        t: f64,
        spice: &mut Option<Spice>,
    ) -> Result<(), Box<dyn std::error::Error>> {
        match &mut self.system {
            BaseSystems::Celestial(celestial) => {
                if let Some(spice) = spice {
                    celestial.update(t, spice)?
                } else {
                    return Err(CelestialErrors::SpiceNotFound.into());
                }
            }
            BaseSystems::Basic(_) => {}
        }
        Ok(())
    }
}

impl BodyTrait for Base {
    fn get_name(&self) -> String {
        self.name.clone()
    }
    fn connect_outer_joint(&mut self, joint: &Joint) -> Result<(), BodyErrors> {
        // Check if the joint already exists in outer_joints
        if self.outer_joints.iter().any(|id| *id == joint.name) {
            return Err(BodyErrors::OuterJointExists(
                joint.name.clone(),
                self.name.clone(),
            ));
        }

        // Push the new joint connection
        self.outer_joints.push(joint.name.clone());
        Ok(())
    }
}
