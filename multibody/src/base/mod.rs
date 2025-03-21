use crate::system::Id;

use celestial::{CelestialErrors, CelestialSystem};
use gravity::Gravity;

use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum BaseErrors {
    #[error("base is celestial")]
    BaseIsCelestial,
    #[error("base is not celestial")]
    BaseIsNotCelestial,
    #[error("CelestialError: {0}")]
    CelestialError(#[from] CelestialErrors),
    #[error("joint '{0}' already connected to {1} as an outer joint")]
    OuterJointExists(String, String),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum BaseSystems {
    Basic(Option<Gravity>),
    Celestial(CelestialSystem),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Base {
    pub name: String,
    pub outer_joints: Vec<Id>,
}

impl Base {
    pub fn new() -> Base {
        Base {
            name: "base".to_string(),
            outer_joints: Vec::new(),
        }
    }

    pub fn connect_outer_joint(&mut self, outer_joint: Id) -> Result<(), BaseErrors> {
        if self.outer_joints.contains(&outer_joint) {
            return Err(BaseErrors::OuterJointExists(
                self.name.clone(),
                outer_joint.to_string(),
            ));
        }
        self.outer_joints.push(outer_joint);
        Ok(())
    }

    /// Builder method for adding a celestial system, ideally used when compiling the system
    pub fn with_celestial_system(mut self, celestial: CelestialSystem) {
        self.system = BaseSystems::Celestial(celestial);
        self
    }

    /// Setter method for adding a celestial system, ideally used interactively from the REPL
    pub fn set_celestial_system(&mut self, celestial: CelestialSystem) {
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

    pub fn update(&mut self, t: f64) -> Result<(), Box<dyn std::error::Error>> {
        match &mut self.system {
            BaseSystems::Celestial(celestial) => celestial.update(t)?,
            BaseSystems::Basic(_) => {}
        }
        Ok(())
    }
}
