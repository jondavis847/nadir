use uuid::Uuid;

pub mod base;
pub mod body;
pub mod joints;
pub mod mass_properties;

use crate::ui::dummies::DummyComponent;
use base::{Base, BaseErrors};
use body::{Body, BodyErrors};
use joints::{
    revolute::{Revolute, RevoluteErrors},
    Joint,
};

#[derive(Debug, Clone)]
pub struct MultibodyMeta {
    component_id: Uuid,
    dummy_id: Uuid,
    from_id: Option<Uuid>,
    name: String,
    node_id: Uuid,
    system_id: Option<usize>,
    to_id: Vec<Uuid>,
}

impl MultibodyMeta {
    pub fn new(component_id: Uuid, dummy_id: Uuid, name: String, node_id: Uuid) -> Self {
        let from_id = None;
        let system_id = None;
        let to_id = Vec::new();
        Self {
            component_id,
            dummy_id,
            from_id,
            name,
            node_id,
            system_id,
            to_id,
        }
    }
}

pub trait MultibodyTrait {
    fn connect_from(&mut self, id: Uuid);
    fn connect_to(&mut self, id: Uuid);
    fn delete_from(&mut self);
    fn delete_to(&mut self, id: Uuid);
    fn get_component_id(&self) -> Uuid;
    fn get_dummy_id(&self) -> Uuid;
    fn get_from_id(&self) -> Option<Uuid>;
    fn get_name(&self) -> &str;
    fn get_node_id(&self) -> Uuid;
    fn get_to_id(&self) -> &Vec<Uuid>;
    fn inherit_from(&mut self, dummy: &DummyComponent);
    fn set_component_id(&mut self, id: Uuid);
    fn set_name(&mut self, name: String);
    fn set_node_id(&mut self, id: Uuid);
    fn set_system_id(&mut self, id: usize);
}

pub enum MultibodyErrors {
    Base(BaseErrors),
    Body(BodyErrors),
    Revolute(RevoluteErrors),
}

#[derive(Debug, Clone)]
pub enum MultibodyComponent {
    Base(Base),
    Body(Body),
    Joint(Joint),
}

impl MultibodyTrait for MultibodyComponent {
    fn connect_from(&mut self, from_id: Uuid) {
        match self {
            MultibodyComponent::Base(base) => base.connect_from(from_id),
            MultibodyComponent::Body(body) => body.connect_from(from_id),
            MultibodyComponent::Joint(joint) => joint.connect_from(from_id),
        }
    }
    fn connect_to(&mut self, to_id: Uuid) {
        match self {
            MultibodyComponent::Base(base) => base.connect_to(to_id),
            MultibodyComponent::Body(body) => body.connect_to(to_id),
            MultibodyComponent::Joint(joint) => joint.connect_to(to_id),
        }
    }

    fn delete_from(&mut self) {
        match self {
            MultibodyComponent::Base(base) => base.delete_from(),
            MultibodyComponent::Body(body) => body.delete_from(),
            MultibodyComponent::Joint(joint) => joint.delete_from(),
        }
    }

    fn delete_to(&mut self, id: Uuid) {
        match self {
            MultibodyComponent::Base(base) => base.delete_to(id),
            MultibodyComponent::Body(body) => body.delete_to(id),
            MultibodyComponent::Joint(joint) => joint.delete_to(id),
        }
    }

    fn get_component_id(&self) -> Uuid {
        match self {
            MultibodyComponent::Base(base) => base.get_component_id(),
            MultibodyComponent::Body(body) => body.get_component_id(),
            MultibodyComponent::Joint(joint) => joint.get_component_id(),
        }
    }

    fn get_dummy_id(&self) -> Uuid {
        match self {
            MultibodyComponent::Base(base) => base.get_dummy_id(),
            MultibodyComponent::Body(body) => body.get_dummy_id(),
            MultibodyComponent::Joint(joint) => joint.get_dummy_id(),
        }
    }

    fn get_from_id(&self) -> Option<Uuid> {
        match self {
            MultibodyComponent::Base(base) => base.get_from_id(),
            MultibodyComponent::Body(body) => body.get_from_id(),
            MultibodyComponent::Joint(joint) => joint.get_from_id(),
        }
    }

    fn get_name(&self) -> &str {
        match self {
            MultibodyComponent::Base(base) => base.get_name(),
            MultibodyComponent::Body(body) => body.get_name(),
            MultibodyComponent::Joint(joint) => joint.get_name(),
        }
    }

    fn get_node_id(&self) -> Uuid {
        match self {
            MultibodyComponent::Base(base) => base.get_node_id(),
            MultibodyComponent::Body(body) => body.get_node_id(),
            MultibodyComponent::Joint(joint) => joint.get_node_id(),
        }
    }

    fn get_to_id(&self) -> &Vec<Uuid> {
        match self {
            MultibodyComponent::Base(base) => base.get_to_id(),
            MultibodyComponent::Body(body) => body.get_to_id(),
            MultibodyComponent::Joint(joint) => joint.get_to_id(),
        }
    }

    fn inherit_from(&mut self, dummy: &DummyComponent) {
        match self {
            MultibodyComponent::Base(base) => base.inherit_from(dummy),
            MultibodyComponent::Body(body) => body.inherit_from(dummy),
            MultibodyComponent::Joint(joint) => joint.inherit_from(dummy),
        }
    }

    fn set_component_id(&mut self, id: Uuid) {
        match self {
            MultibodyComponent::Base(base) => base.set_component_id(id),
            MultibodyComponent::Body(body) => body.set_component_id(id),
            MultibodyComponent::Joint(joint) => joint.set_component_id(id),
        }
    }

    fn set_name(&mut self, name: String) {
        match self {
            MultibodyComponent::Base(base) => base.set_name(name),
            MultibodyComponent::Body(body) => body.set_name(name),
            MultibodyComponent::Joint(joint) => joint.set_name(name),
        }
    }

    fn set_node_id(&mut self, id: Uuid) {
        match self {
            MultibodyComponent::Base(base) => base.set_node_id(id),
            MultibodyComponent::Body(body) => body.set_node_id(id),
            MultibodyComponent::Joint(joint) => joint.set_node_id(id),
        }
    }

    fn set_system_id(&mut self, id: usize) {
        match self {
            MultibodyComponent::Base(base) => base.set_system_id(id),
            MultibodyComponent::Body(body) => body.set_system_id(id),
            MultibodyComponent::Joint(joint) => joint.set_system_id(id),
        }
    }
}

impl MultibodyComponent {
    pub fn from_dummy(
        component_id: Uuid,
        dummy: &DummyComponent,
        node_id: Uuid,
    ) -> Result<Self, MultibodyErrors> {
        let component = match dummy {
            DummyComponent::Base(dummy) => {
                let base = match Base::from_dummy(component_id, dummy, node_id) {
                    Ok(base) => base,
                    Err(error) => return Err(MultibodyErrors::Base(error)),
                };
                MultibodyComponent::Base(base)
            }
            DummyComponent::Body(dummy) => {
                let body = match Body::from_dummy(component_id, dummy, node_id) {
                    Ok(body) => body,
                    Err(error) => return Err(MultibodyErrors::Body(error)),
                };
                MultibodyComponent::Body(body)
            }
            DummyComponent::Revolute(dummy) => {
                let revolute = match Revolute::from_dummy(component_id, dummy, node_id) {
                    Ok(revolute) => revolute,
                    Err(error) => return Err(MultibodyErrors::Revolute(error)),
                };
                MultibodyComponent::Joint(Joint::Revolute(revolute))
            }
        };
        Ok(component)
    }
}

#[derive(Debug, Clone)]
pub struct MultibodySystem {
    bodies: Vec<MultibodyComponent>, // MultibodyComponent since its both base and bodies
    joints: Vec<Joint>,
}

impl MultibodySystem {
    pub fn new(bodies: Vec<MultibodyComponent>, joints: Vec<Joint>) -> Self {
        Self { bodies, joints }
    }
}


#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}
