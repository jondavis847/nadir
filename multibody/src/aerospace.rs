use aerospace::gravity::Gravity;
use uuid::Uuid;
use serde::{Serialize, Deserialize};

use crate::MultibodyTrait;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MultibodyGravity {
    id: Uuid,
    name: String,
    pub gravity: Gravity,
}

impl MultibodyGravity {
    pub fn new(name: &str, gravity: Gravity) -> Self {
        Self {
            id: Uuid::new_v4(),
            name: name.to_string(),
            gravity,
        }
    }
}

impl MultibodyTrait for MultibodyGravity {
    fn get_id(&self) -> &Uuid {
        &self.id
    }
    fn get_name(&self) -> &str {
        &self.name
    }
    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
