pub mod errors;
pub mod floating;
pub mod joint_sim;
pub mod joint_state;
pub mod joint_transforms;
pub mod prismatic;
pub mod revolute;

use crate::{
    body::BodyConnection,
    result::{MultibodyResultTrait, ResultEntry},
};

use super::{
    body::{Body, BodyTrait},
    MultibodyTrait,
};
use errors::JointErrors;
use floating::{Floating, FloatingResult};
use prismatic::{Prismatic, PrismaticResult};
use revolute::{Revolute, RevoluteResult};
use serde::{Deserialize, Serialize};
use spatial_algebra::SpatialInertia;
use std::fmt;
use transforms::Transform;
use uuid::Uuid;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Joint {
    Floating(Floating),
    Prismatic(Prismatic),
    Revolute(Revolute),
    //Spherical,
}

impl fmt::Display for Joint {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Joint::Floating(_) => {
                writeln!(f, "Joint: Floating")
                //writeln!(f, "\t{}", prismatic)
            }
            Joint::Prismatic(_) => {
                writeln!(f, "Joint: Prismatic")
                //writeln!(f, "\t{}", prismatic)
            }
            Joint::Revolute(_) => {
                writeln!(f, "Joint: Revolute")
                //writeln!(f, "\t{}", revolute)
            }
        }
    }
}

impl From<Revolute> for Joint {
    fn from(revolute: Revolute) -> Self {
        Joint::Revolute(revolute)
    }
}
impl From<Prismatic> for Joint {
    fn from(prismatic: Prismatic) -> Self {
        Joint::Prismatic(prismatic)
    }
}

pub trait JointTrait: MultibodyTrait {
    fn connect_inner_body<T: BodyTrait>(
        &mut self,
        body: &mut T,
        transform: Transform,
    ) -> Result<(), JointErrors>;

    fn connect_outer_body(
        &mut self,
        body: &mut Body,
        transform: Transform,
    ) -> Result<(), JointErrors>;

    fn delete_inner_body_id(&mut self);
    fn delete_outer_body_id(&mut self);
    fn get_connections(&self) -> &JointConnection;
    fn get_connections_mut(&mut self) -> &mut JointConnection;
    fn get_inner_body_id(&self) -> Option<&Uuid>;
    fn get_outer_body_id(&self) -> Option<&Uuid>;
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct JointCommon {
    pub id: Uuid,
    pub name: String,
    pub connection: JointConnection,
    pub mass_properties: Option<SpatialInertia>,
}

impl JointCommon {
    pub fn new(name: &str) -> Self {
        Self {
            id: Uuid::new_v4(),
            name: name.to_string(),
            connection: JointConnection::default(),
            mass_properties: None,
        }
    }
}

impl MultibodyTrait for Joint {
    fn get_id(&self) -> &Uuid {
        match self {
            Joint::Floating(joint) => joint.get_id(),
            Joint::Prismatic(joint) => joint.get_id(),
            Joint::Revolute(joint) => joint.get_id(),
        }
    }
    fn get_name(&self) -> &str {
        match self {
            Joint::Floating(joint) => joint.get_name(),
            Joint::Prismatic(joint) => joint.get_name(),
            Joint::Revolute(joint) => joint.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            Joint::Floating(joint) => joint.set_name(name),
            Joint::Prismatic(joint) => joint.set_name(name),
            Joint::Revolute(joint) => joint.set_name(name),
        }
    }
}

impl JointTrait for Joint {
    fn connect_inner_body<T: BodyTrait>(
        &mut self,
        body: &mut T,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        match self {
            Joint::Floating(joint) => joint.connect_inner_body(body, transform),
            Joint::Prismatic(joint) => joint.connect_inner_body(body, transform),
            Joint::Revolute(joint) => joint.connect_inner_body(body, transform),
        }
    }

    fn connect_outer_body(
        &mut self,
        body: &mut Body,
        transform: Transform,
    ) -> Result<(), JointErrors> {
        match self {
            Joint::Floating(joint) => joint.connect_outer_body(body, transform),
            Joint::Prismatic(joint) => joint.connect_outer_body(body, transform),
            Joint::Revolute(joint) => joint.connect_outer_body(body, transform),
        }
    }

    fn delete_inner_body_id(&mut self) {
        match self {
            Joint::Floating(joint) => joint.delete_inner_body_id(),
            Joint::Prismatic(joint) => joint.delete_inner_body_id(),
            Joint::Revolute(joint) => joint.delete_inner_body_id(),
        }
    }
    fn delete_outer_body_id(&mut self) {
        match self {
            Joint::Floating(joint) => joint.delete_outer_body_id(),
            Joint::Prismatic(joint) => joint.delete_outer_body_id(),
            Joint::Revolute(joint) => joint.delete_outer_body_id(),
        }
    }

    fn get_connections(&self) -> &JointConnection {
        match self {
            Joint::Floating(joint) => joint.get_connections(),
            Joint::Prismatic(joint) => joint.get_connections(),
            Joint::Revolute(joint) => joint.get_connections(),
        }
    }

    fn get_connections_mut(&mut self) -> &mut JointConnection {
        match self {
            Joint::Floating(joint) => joint.get_connections_mut(),
            Joint::Prismatic(joint) => joint.get_connections_mut(),
            Joint::Revolute(joint) => joint.get_connections_mut(),
        }
    }

    fn get_inner_body_id(&self) -> Option<&Uuid> {
        match self {
            Joint::Floating(joint) => joint.get_inner_body_id(),
            Joint::Prismatic(joint) => joint.get_inner_body_id(),
            Joint::Revolute(joint) => joint.get_inner_body_id(),
        }
    }

    fn get_outer_body_id(&self) -> Option<&Uuid> {
        match self {
            Joint::Floating(joint) => joint.get_outer_body_id(),
            Joint::Prismatic(joint) => joint.get_outer_body_id(),
            Joint::Revolute(joint) => joint.get_outer_body_id(),
        }
    }
}

// We choose to keep the transform information with the joint rather than the body.
// This is because bodies can have multiple outer joints where as a joint only has one inner or outer body.
// If it was in the body, and I want the the transform from parent body to this body,
// then I need the transform from parent body to this inner joint.
// To get that transform with that info in the parent body, we would need to search it's
// outer joints for this inner joint and apply that transform.
// If the transform is already in the joint, then no searching.
// We also choose to make the transform be from the body frame to the joint frame.
// This is because the location of things like actuators or other components are typically expressed in the body frame.

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct JointConnection {
    pub inner_body: Option<BodyConnection>,
    pub outer_body: Option<BodyConnection>,
}

#[derive(Debug, Default, Clone, Copy, Serialize, Deserialize)]
pub struct JointParameters {
    pub constant_force: f64,
    pub damping: f64,
    pub spring_constant: f64,
}

impl JointParameters {
    pub fn new(constant_force: f64, damping: f64, spring_constant: f64) -> Self {
        Self {
            constant_force,
            damping,
            spring_constant,
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum JointResult {
    Floating(FloatingResult),
    Prismatic(PrismaticResult),
    Revolute(RevoluteResult),
}

impl MultibodyResultTrait for JointResult {
    fn get_state_names(&self) -> Vec<&'static str> {
        match self {
            JointResult::Floating(result) => result.get_state_names(),
            JointResult::Prismatic(result) => result.get_state_names(),
            JointResult::Revolute(result) => result.get_state_names(),
        }
    }

    fn get_result_entry(&self) -> ResultEntry {
        match self {
            JointResult::Floating(result) => result.get_result_entry(),
            JointResult::Prismatic(result) => result.get_result_entry(),
            JointResult::Revolute(result) => result.get_result_entry(),
        }
    }
}
