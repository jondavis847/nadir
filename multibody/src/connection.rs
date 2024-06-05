use super::{body::Bodies, joint::Joint, MultibodyTrait};
use sim_value::SimValue;
use std::cell::RefCell;
use std::fmt;
use std::rc::Rc;
use transforms::Transform;

#[derive(Debug, Clone, Copy)]
pub enum Port {
    Body,
    Joint,
}

#[derive(Clone)]
pub struct Connection<T>
where
    T: SimValue,
{
    joint: Rc<RefCell<Joint<T>>>,
    inner_body: Rc<RefCell<Bodies<T>>>,
    inner_transform: Transform<T>,
    outer_body: Rc<RefCell<Bodies<T>>>,
    outer_transform: Transform<T>,
}

impl<T> Connection<T>
where
    T: SimValue,
{
    pub fn new(
        joint: Rc<RefCell<Joint<T>>>,
        inner_body: Rc<RefCell<Bodies<T>>>,
        inner_transform: Transform<T>,
        outer_body: Rc<RefCell<Bodies<T>>>,
        outer_transform: Transform<T>,
    ) -> Self {
        Self {
            joint,
            inner_body,
            inner_transform,
            outer_body,
            outer_transform,
        }
    }
}

impl<T> fmt::Debug for Connection<T>
where
    T: SimValue,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let joint = self.joint.borrow();
        let inner_body = self.inner_body.borrow();
        let outer_body = self.outer_body.borrow();

        f.debug_struct("Connection")
            .field("joint_name", &joint.get_name())
            .field("inner_body_name", &inner_body.get_name())            
            .field("outer_body_name", &outer_body.get_name())
            .finish()
    }
}

#[derive(Debug, Clone)]
pub enum ConnectionErrors {
    BodyInnerAlreadyExists,
    ComponentNotFound,
    JointInnerAlreadyExists,
    JointOuterAlreadyExists,
    NothingBeforeBase,
}
