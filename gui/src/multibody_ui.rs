// see the dependency multibody for actual multibody dynamics
// this is just the ui part of multibody

use multibody::{base::Base, body::Body, joint::Joint, MultibodyTrait};

#[derive(Debug,Clone)]
pub enum MultibodyComponent {
    Base(Base),
    Body(Body),
    Joint(Joint),
}

impl MultibodyComponent {
    pub fn get_name(&self) -> &str {
        match self {
            MultibodyComponent::Base(base) => base.get_name(),
            MultibodyComponent::Body(body) => body.get_name(),
            MultibodyComponent::Joint(joint) => joint.get_name(),
        }
    }
}

pub enum BodyField {
    Name,
    Mass,
    Cmx,
    Cmy,
    Cmz,
    Ixx,
    Iyy,
    Izz,
    Ixy,
    Ixz,
    Iyz,
}

pub enum RevoluteField {
    Name,
    ConstantForce,
    Dampening,
    SpringConstant,
    Omega,
    Theta,
}
