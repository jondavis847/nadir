use std::{cell::RefCell, rc::{Rc, Weak}};

use crate::joint::{Joint, JointRef};

use super::body::{BodyErrors, BodyTrait};
use aerospace::{
    celestial_system::{CelestialErrors, CelestialSystem},
    gravity::Gravity,
};
use serde::{Deserialize, Serialize};
use spice::Spice;
use thiserror::Error;
use transforms::Transform;

#[derive(Debug, Error)]
pub enum BaseErrors {
    #[error("base is celestial")]
    BaseIsCelestial,
    #[error("CelestialError: {0}")]
    CelestialError(#[from] CelestialErrors),
}

pub type BaseRef = Rc<RefCell<Base>>;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum BaseSystems {
    Basic(Option<Gravity>),
    Celestial(CelestialSystem),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Base {
    pub name: String,
    pub system: BaseSystems,
    pub outer_joints: Vec<Weak<RefCell<Joint>>>,
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
    fn connect_outer_joint(&mut self, joint: &JointRef) -> Result<(), BodyErrors> {
        // Iterate over the existing outer joints
        for jointref in &self.outer_joints {
            if let Some(existing_joint) = jointref.upgrade() {
                // Check if the existing joint and the new joint point to the same allocation
                if Rc::ptr_eq(&existing_joint, joint) {
                    return Err(BodyErrors::OuterJointExists(
                        existing_joint.borrow().name.clone(),
                        self.name.clone(),
                    ));
                }
            }
        }
    
        // If no match is found, add the new joint to outer_joints
        self.outer_joints.push(Rc::downgrade(&joint.clone()));
        Ok(())
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BaseConnection {
    pub base: Rc<RefCell<Base>>,
    pub transform: Transform,
}
impl BaseConnection {
    pub fn new(base: Rc<RefCell<Base>>, transform: Transform) -> Self {
        Self { base, transform }
    }
}
