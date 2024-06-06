use super::{
    body::{BodyEnum, BodyErrors, BodyJointConnection, BodyRef, BodyTrait},
    joint::JointRef,
    MultibodyTrait,
};
use transforms::Transform;

use sim_value::SimValue;
use std::cell::RefCell;
use std::rc::Rc;

pub type BaseRef<T> = Rc<RefCell<Base<T>>>;
#[derive(Debug, Clone)]
pub struct Base<T>
where
    T: SimValue,
{
    name: String,
    outer_joints: Vec<BodyJointConnection<T>>,
}

impl<T> Base<T>
where
    T: SimValue,
{
    pub fn new(name: &str) -> Result<BodyRef<T>, BodyErrors>
    where
        T: SimValue,
    {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(Rc::new(RefCell::new(BodyEnum::Base(Self {            
            name: name.to_string(),
            outer_joints: Vec::new(),
        }))))
    }
}

pub enum BaseErrors {}

impl<T> BodyTrait<T> for Base<T>
where
    T: SimValue,
{
    fn connect_inner_joint(
        &mut self,
        _jointref: JointRef<T>,
        _transform: Transform<T>,
    ) -> Result<(), BodyErrors> {
        Err(BodyErrors::NoBaseInnerConnection)        
    }

    fn connect_outer_joint(
        &mut self,
        jointref: JointRef<T>,
        transform: Transform<T>,
    ) -> Result<(), BodyErrors> {
        // Borrow the joint and get its name
        let joint_name = jointref.borrow().get_name().to_string();

        // Check if the joint already exists in outer_joints
        if self
            .outer_joints
            .iter()
            .any(|connection| connection.component.borrow().get_name() == joint_name)
        {
            return Err(BodyErrors::OuterJointExists);
        }

        // Push the new joint connection
        self.outer_joints
            .push(BodyJointConnection::new(jointref, transform));
        Ok(())
    }

    fn delete_inner_joint(&mut self) {
        //nothing to do
    }

    fn delete_outer_joint(&mut self, jointref: JointRef<T>) {
        self.outer_joints
            .retain(|connection| !Rc::ptr_eq(&connection.component, &jointref));
    }
}
impl<T> MultibodyTrait for Base<T> where T: SimValue {
    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
