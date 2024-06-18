use super::{base::Base,body::Body,joint::JointEnum, MultibodyTrait};

#[derive(Debug, Clone)]
pub enum MultibodyComponent {
    Base(Base),
    Body(Body),
    Joint(JointEnum),
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
