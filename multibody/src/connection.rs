use sim_value::SimValue;
use transforms::Transform;

#[derive(Debug, Clone, Copy)]
pub enum Port {
    Body,
    Joint,
}

#[derive(Debug, Clone, Copy)]
pub struct Connection<T>
where
    T: SimValue,
{
    id: usize,
    inner: Port,
    outer: Port,
    transform: Transform<T>,
}

impl<T> Connection<T>
where
    T: SimValue,
{
    pub fn new(id: usize, inner: Port, outer: Port, transform: Transform<T>) -> Self {
        Self {
            id,
            inner,
            outer,
            transform,
        }
    }
}

#[derive(Debug, Clone)]
pub enum ConnectionErrors {
    ComponentNotFound(String),
    BodyInnerAlreadyExists,
    BodyToBody,
    JointInnerAlreadyExists,
    JointOuterAlreadyExists,
    JointToJoint,
    NothingBeforeBase,
}
