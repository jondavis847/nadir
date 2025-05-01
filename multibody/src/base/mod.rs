use std::{
    cell::RefCell,
    rc::{Rc, Weak},
};

use crate::{
    body::BodyConnectionBuilder,
    joint::{Joint, JointBuilder, JointErrors},
    system::Id,
};

use celestial::{CelestialErrors, CelestialSystem, CelestialSystemBuilder};
use gravity::Gravity;

use serde::{Deserialize, Serialize};
use thiserror::Error;
use transforms::Transform;

#[derive(Debug, Error)]
pub enum BaseErrors {
    #[error("base is celestial, delete it if needed with .delete_celestial()")]
    BaseIsCelestial,
    #[error("base is not celestial")]
    BaseIsNotCelestial,
    #[error("CelestialError: {0}")]
    CelestialError(#[from] CelestialErrors),
    #[error("{0}")]
    Joint(#[from] JointErrors),
    #[error("joint '{0}' already connected to {1} as an outer joint")]
    OuterJointExists(String, String),
}

#[derive(Debug)]
pub enum BaseSystems {
    Basic(Option<Gravity>),
    Celestial(CelestialSystem),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum BaseSystemsBuilder {
    Basic(Option<Gravity>),
    Celestial(CelestialSystemBuilder),
}

impl From<&BaseSystemsBuilder> for BaseSystems {
    fn from(builder: &BaseSystemsBuilder) -> BaseSystems {
        match builder {
            BaseSystemsBuilder::Basic(g) => BaseSystems::Basic(g.clone()),
            BaseSystemsBuilder::Celestial(c) => BaseSystems::Celestial(CelestialSystem::from(c)),
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct BaseBuilder {
    pub id: Id,
    pub name: String,
    pub outer_joints: Vec<Id>,
    pub system: BaseSystemsBuilder,
}

impl BaseBuilder {
    pub fn new(id: Id) -> Self {
        Self {
            id,
            name: "base".to_string(),
            outer_joints: Vec::new(),
            system: BaseSystemsBuilder::Basic(None),
        }
    }

    pub fn connect_outer_joint(
        &mut self,
        outer_joint: &mut JointBuilder,
        transform: Transform,
    ) -> Result<(), BaseErrors> {
        if self.outer_joints.contains(&outer_joint.id) {
            return Err(BaseErrors::OuterJointExists(
                self.name.clone(),
                outer_joint.name.clone(),
            ));
        }
        if outer_joint.connections.inner_body.is_some() {
            return Err(JointErrors::InnerBodyExists(outer_joint.name.clone()).into());
        }
        self.outer_joints.push(outer_joint.id);
        outer_joint.connections.inner_body = Some(BodyConnectionBuilder::new(self.id, transform));
        Ok(())
    }

    /// Builder method for adding a celestial system, ideally used when compiling the system
    pub fn with_celestial(mut self, celestial: CelestialSystemBuilder) -> Self {
        self.system = BaseSystemsBuilder::Celestial(celestial);
        self
    }

    /// Setter method for adding a celestial system, ideally used interactively from the REPL
    pub fn set_celestial(&mut self, celestial: CelestialSystemBuilder) {
        self.system = BaseSystemsBuilder::Celestial(celestial);
    }

    pub fn set_basic(&mut self, gravity: Option<Gravity>) -> Result<(), BaseErrors> {
        self.system = BaseSystemsBuilder::Basic(gravity);
        Ok(())
    }
}

#[derive(Debug)]
pub struct Base {
    pub outer_joints: Vec<Weak<RefCell<Joint>>>,
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
            system: BaseSystems::from(&builder.system),
        }
    }
}
