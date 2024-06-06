pub mod base;
pub mod body;
pub mod joint;
pub mod mass_properties;

use base::{Base, BaseErrors};
use body::{Body, BodyEnum, BodyErrors, BodyRef, BodyTrait};
use joint::{revolute::RevoluteErrors, Joint, JointRef, JointTrait};

pub enum MultibodyErrors {
    Base(BaseErrors),
    BaseAlreadyExists,
    BaseMissingOuterJoint,
    Body(BodyErrors),
    BodyMissingInnerJoint(BodyRef),
    JointMissingInnerBody(JointRef),
    JointMissingOuterBody(JointRef),
    NameTaken,
    NoBaseFound,
    Revolute(RevoluteErrors),
    TooManyBasesFound,
}

pub trait MultibodyTrait {
    fn get_name(&self) -> &str;
    fn set_name(&mut self, name: String);
}

#[derive(Debug, Clone)]
pub enum MultibodyComponent {
    Base(Base),
    Body(Body),
    Joint(Joint),
}

impl MultibodyTrait for MultibodyComponent {
    fn get_name(&self) -> &str {
        match self {
            MultibodyComponent::Base(base) => base.get_name(),
            MultibodyComponent::Body(body) => body.get_name(),
            MultibodyComponent::Joint(joint) => joint.get_name(),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            MultibodyComponent::Base(base) => base.set_name(name),
            MultibodyComponent::Body(body) => body.set_name(name),
            MultibodyComponent::Joint(joint) => joint.set_name(name),
        }
    }
}

#[derive(Debug, Clone)]
pub struct MultibodySystem {
    bodies: Vec<BodyRef>, // must be Body
    joints: Vec<JointRef>,
}

impl MultibodySystem {
    pub fn new() -> Self {
        Self {
            bodies: Vec::new(),
            joints: Vec::new(),
        }
    }

    pub fn add_body(&mut self, bodyref: BodyRef) -> Result<(), MultibodyErrors> {
        // return if this is a Base and we already have a Base
        if matches!(*bodyref.borrow(), BodyEnum::Base(_)) {
            if let Some(_) = self.bodies.iter().find(|body| {
                let body = body.borrow();
                matches!(*body, BodyEnum::Base(_))
            }) {
                return Err(MultibodyErrors::BaseAlreadyExists);
            }
        }

        // Return if a component with this name already exists
        let name = bodyref.borrow().get_name().to_string(); // Clone the name to avoid multiple borrow

        if self
            .bodies
            .iter()
            .any(|body| body.borrow().get_name() == name)
        {
            return Err(MultibodyErrors::NameTaken);
        }

        if self
            .joints
            .iter()
            .any(|joint| joint.borrow().get_name() == name)
        {
            return Err(MultibodyErrors::NameTaken);
        }

        self.bodies.push(bodyref);
        Ok(())
    }

    pub fn add_joint(&mut self, jointref: JointRef) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        let name = jointref.borrow().get_name().to_string(); // Clone the name to avoid multiple borrow

        if self
            .bodies
            .iter()
            .any(|body| body.borrow().get_name() == name)
        {
            return Err(MultibodyErrors::NameTaken);
        }

        if self
            .joints
            .iter()
            .any(|joint| joint.borrow().get_name() == name)
        {
            return Err(MultibodyErrors::NameTaken);
        }
        self.joints.push(jointref);
        Ok(())
    }

    pub fn find_body_by_name(&self, name: &str) -> Option<BodyRef> {
        self.bodies
            .iter()
            .find(|joint| joint.borrow().get_name() == name)
            .cloned()
    }

    pub fn find_joint_by_name(&self, name: &str) -> Option<JointRef> {
        self.joints
            .iter()
            .find(|joint| joint.borrow().get_name() == name)
            .cloned()
    }

    pub fn validate(&mut self) -> Result<(), MultibodyErrors> {
        // check that there's at least 1 and only 1 base
        let num_bases = self
            .bodies
            .iter()
            .filter(|body_ref| match &*body_ref.borrow() {
                BodyEnum::Base(_) => true,
                _ => false,
            })
            .count();

        if num_bases == 0 {
            return Err(MultibodyErrors::NoBaseFound);
        };
        if num_bases > 1 {
            return Err(MultibodyErrors::TooManyBasesFound);
        };

        println!("Found exactly 1 base.");

        // check that the base has an outer joint
        if let Some(base) = self.bodies.iter().find(|bodyref| {
            let body = bodyref.borrow();
            match &*body {
                BodyEnum::Base(base) => base.get_outer_joints().is_empty(),
                BodyEnum::Body(_) => false,
            }
        }) {
            return Err(MultibodyErrors::BaseMissingOuterJoint);
        }

        println!("Base has at least 1 outer joint.");

        // check that every body has an inner joint
        if let Some(body) = self.bodies.iter().find(|bodyref| {
            let body = bodyref.borrow();
            match &*body {
                BodyEnum::Base(_) => false,
                BodyEnum::Body(body) => body.get_inner_joint().is_none(),
            }
        }) {
            return Err(MultibodyErrors::BodyMissingInnerJoint(body.clone()));
        }

        println!("All bodies have an inner joint.");

        // check that every joint has an inner and outer body connection
        if let Some(joint) = self
            .joints
            .iter()
            .find(|joint| joint.get_inner_body().is_none())
        {
            return Err(MultibodyErrors::JointMissingInnerBody(joint.clone()));
        }

        println!("All joints have an inner body.");

        if let Some(joint) = self
            .joints
            .iter()
            .find(|joint| joint.get_outer_body().is_none())
        {
            return Err(MultibodyErrors::JointMissingOuterBody(joint.clone()));
        }

        println!("All joints have an outer body.");

        Ok(())
    }
}
