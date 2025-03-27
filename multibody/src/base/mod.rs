use std::{cell::RefCell, rc::Rc};

use crate::{joint::JointRef, system::Id};

use celestial::{CelestialErrors, CelestialSystem};
use gravity::Gravity;

use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum BaseErrors {
    #[error("base is celestial, delete it if needed with .delete_celestial()")]
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
pub struct BaseBuilder {
    pub name: String,
    pub outer_joints: Vec<Id>,
    pub system: BaseSystems,
}

impl BaseBuilder {
    pub fn new() -> Self {
        Self {
            name: "base".to_string(),
            outer_joints: Vec::new(),
            system: BaseSystems::Basic(None),
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
    pub fn with_celestial(mut self, celestial: CelestialSystem) -> Self {
        self.system = BaseSystems::Celestial(celestial);
        self
    }

    /// Setter method for adding a celestial system, ideally used interactively from the REPL
    pub fn set_celestial(&mut self, celestial: CelestialSystem) {
        self.system = BaseSystems::Celestial(celestial);
    }

    pub fn set_basic(&mut self, gravity: Option<Gravity>) -> Result<(), BaseErrors> {
        self.system = BaseSystems::Basic(gravity);
        Ok(())
    }
}

#[derive(Clone, Debug)]
pub struct Base {
    pub outer_joints: Vec<JointRef>,
    pub system: BaseSystems,
}

impl Base {
    pub fn update(&mut self, t: f64) -> Result<(), Box<dyn std::error::Error>> {
        match &mut self.system {
            BaseSystems::Celestial(celestial) => celestial.update(t)?,
            BaseSystems::Basic(_) => {}
        }
        Ok(())
    }
}

pub type BaseRef = Rc<RefCell<Base>>;

impl From<&BaseBuilder> for Base {
    fn from(builder: &BaseBuilder) -> Self {
        Self {
            outer_joints: Vec::new(),
            system: builder.system.clone(),
        }
    }
}
