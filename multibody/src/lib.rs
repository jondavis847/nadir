use sim_value::SimValue;

pub mod base;
pub mod body;
pub mod joint;
pub mod mass_properties;

use base::{Base, BaseErrors};
use body::{BodyEnum, Body, BodyErrors, BodyRef};
use joint::{revolute::RevoluteErrors, Joint, JointRef};
use transforms::Transform;

pub enum MultibodyErrors {
    Base(BaseErrors),
    BaseAlreadyExists,
    Body(BodyErrors),
    NameTaken,
    Revolute(RevoluteErrors),
}

pub trait MultibodyTrait {
    fn get_name(&self) -> &str;
    fn set_name(&mut self, name: String);
}

#[derive(Debug, Clone)]
pub enum MultibodyComponent<T>
where
    T: SimValue,
{
    Base(Base<T>),
    Body(Body<T>),
    Joint(Joint<T>),
}

impl<T> MultibodyTrait for MultibodyComponent<T>
where
    T: SimValue,
{
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
pub struct MultibodySystem<T>
where
    T: SimValue,
{
    BodyEnum: Vec<BodyRef<T>>, // must be Body
    joints: Vec<JointRef<T>>,
}

impl<T> MultibodySystem<T>
where
    T: SimValue,
{
    pub fn new() -> Self {
        Self {
            BodyEnum: Vec::new(),
            joints: Vec::new(),
        }
    }

    pub fn add_body(&mut self, bodyref: BodyRef<T>) -> Result<(), MultibodyErrors> {
        // return if this is a Base and we already have a Base
        if matches!(*bodyref.borrow(), BodyEnum::Base(_)) {
            if let Some(_) = self.BodyEnum.iter().find(|body| {
                let body = body.borrow();
                matches!(*body, BodyEnum::Base(_))
            }) {
                return Err(MultibodyErrors::BaseAlreadyExists);
            }
        }

        // Return if a component with this name already exists
        let name = bodyref.borrow().get_name().to_string(); // Clone the name to avoid multiple borrow

        if self
            .BodyEnum
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

        self.BodyEnum.push(bodyref);
        Ok(())
    }

    pub fn add_joint(&mut self, jointref: JointRef<T>) -> Result<(), MultibodyErrors> {
        // Return if a component with this name already exists
        let name = jointref.borrow().get_name().to_string(); // Clone the name to avoid multiple borrow

        if self
            .BodyEnum
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

    pub fn find_body_by_name(&self, name: &str) -> Option<BodyRef<T>> {
        self.BodyEnum
            .iter()
            .find(|joint| joint.borrow().get_name() == name)
            .cloned()
    }

    pub fn find_joint_by_name(&self, name: &str) -> Option<JointRef<T>> {
        self.joints
            .iter()
            .find(|joint| joint.borrow().get_name() == name)
            .cloned()
    }

    pub fn sort(&mut self) {}
}
