use sim_value::SimValue;
use uuid::Uuid;

pub mod base;
pub mod body;
pub mod joints;
pub mod mass_properties;

use base::{Base, BaseErrors};
use body::{Body, BodyErrors};
use joints::{
    revolute::RevoluteErrors,
    Joint,
};
#[derive(Debug, Clone)]
pub struct MultibodyMeta {
    name: String,
    id: Uuid,
    id_inner: Option<Uuid>,
    id_outer: Vec<Uuid>    
}

impl MultibodyMeta {
    pub fn new(name: String) -> Self {
        
        Self { 
            name: name, 
            id: Uuid::new_v4(),
            id_inner: None,
            id_outer: Vec::new(),
         }
    }
}

pub trait MultibodyTrait {
    fn connect_inner(&mut self, id: Uuid);
    fn connect_outer(&mut self, id: Uuid);
    fn delete_inner(&mut self);
    fn delete_outer(&mut self, id: Uuid);
    fn get_id(&self) -> Uuid;
    fn get_inner_id(&self) -> Option<Uuid>;
    fn get_name(&self) -> &str;
    fn get_outer_id(&self) -> &Vec<Uuid>;    
    fn set_name(&mut self, name: String);
}

pub enum MultibodyErrors {
    Base(BaseErrors),
    Body(BodyErrors),
    Revolute(RevoluteErrors),
}

#[derive(Debug, Clone)]
pub enum MultibodyComponent<T>
where
    T: SimValue,
{
    Base(Base),
    Body(Body<T>),
    Joint(Joint<T>),
}

impl<T> MultibodyTrait for MultibodyComponent<T>
where
    T: SimValue,
{
    fn connect_inner(&mut self, id: Uuid) {
        match self {
            MultibodyComponent::Base(base) => base.connect_inner(id),
            MultibodyComponent::Body(body) => body.connect_inner(id),
            MultibodyComponent::Joint(joint) => joint.connect_inner(id),
        }
    }
    fn connect_outer(&mut self, id: Uuid) {
        match self {
            MultibodyComponent::Base(base) => base.connect_outer(id),
            MultibodyComponent::Body(body) => body.connect_outer(id),
            MultibodyComponent::Joint(joint) => joint.connect_outer(id),
        }
    }

    fn delete_inner(&mut self) {
        match self {
            MultibodyComponent::Base(base) => base.delete_inner(),
            MultibodyComponent::Body(body) => body.delete_inner(),
            MultibodyComponent::Joint(joint) => joint.delete_inner(),
        }
    }

    fn delete_outer(&mut self, id: Uuid) {
        match self {
            MultibodyComponent::Base(base) => base.delete_outer(id),
            MultibodyComponent::Body(body) => body.delete_outer(id),
            MultibodyComponent::Joint(joint) => joint.delete_outer(id),
        }
    }

    fn get_id(&self) -> Uuid {
        match self {
            MultibodyComponent::Base(base) => base.get_id(),
            MultibodyComponent::Body(body) => body.get_id(),
            MultibodyComponent::Joint(joint) => joint.get_id(),
        }
    }

    fn get_inner_id(&self) -> Option<Uuid> {
        match self {
            MultibodyComponent::Base(base) => base.get_inner_id(),
            MultibodyComponent::Body(body) => body.get_inner_id(),
            MultibodyComponent::Joint(joint) => joint.get_inner_id(),
        }
    }

    fn get_name(&self) -> &str {
        match self {
            MultibodyComponent::Base(base) => base.get_name(),
            MultibodyComponent::Body(body) => body.get_name(),
            MultibodyComponent::Joint(joint) => joint.get_name(),
        }
    }

    fn get_outer_id(&self) -> &Vec<Uuid> {
        match self {
            MultibodyComponent::Base(base) => base.get_outer_id(),
            MultibodyComponent::Body(body) => body.get_outer_id(),
            MultibodyComponent::Joint(joint) => joint.get_outer_id(),
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
    bodies: Vec<MultibodyComponent<T>>, // MultibodyComponent since its both base and bodies
    joints: Vec<Joint<T>>,
}

impl<T> MultibodySystem<T>
where
    T: SimValue,
{
    pub fn new(bodies: Vec<MultibodyComponent<T>>, joints: Vec<Joint<T>>) -> Self {
        Self { bodies, joints }
    }
}
