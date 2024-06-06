use super::{
    body::{Bodies, BodyErrors, BodyRef},
    MultibodyTrait,
};

use sim_value::SimValue;
use std::cell::RefCell;
use std::rc::Rc;

pub type BaseRef = Rc<RefCell<Base>>;
#[derive(Debug, Clone)]
pub struct Base {
    name: String,
}

impl Base {
    pub fn new<T>(name: &str) -> Result<BodyRef<T>, BodyErrors>
    where
        T: SimValue,
    {
        if name.is_empty() {
            return Err(BodyErrors::EmptyName);
        }
        Ok(Rc::new(RefCell::new(Bodies::Base(Self {
            name: name.to_string(),
        }))))
    }
}

pub enum BaseErrors {}

impl MultibodyTrait for Base {
    fn get_name(&self) -> &str {
        &self.name
    }

    fn set_name(&mut self, name: String) {
        self.name = name;
    }
}
