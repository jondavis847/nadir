pub mod reaction_wheel;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

use crate::body::{Body, BodyConnection};

#[typetag::serde]
pub trait ActuatorModel: CloneActuatorModel + Debug {
    fn update(&mut self, body: &Body, connection: &BodyConnection);
}
pub trait CloneActuatorModel {
    fn clone_model(&self) -> Box<dyn ActuatorModel>;
}
impl<T> CloneActuatorModel for T
where
    T: ActuatorModel + Clone + 'static,
{
    fn clone_model(&self) -> Box<dyn ActuatorModel> {
        Box::new(self.clone())
    }
}

impl Clone for Box<dyn ActuatorModel> {
    fn clone(&self) -> Self {
        self.clone_model()
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Actuator {
    pub name: String,
    model: Box<dyn ActuatorModel>,
    connection: Option<BodyConnection>,
}
