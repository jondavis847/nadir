use crate::NadirParserErrors;

use super::clone_any::CloneAny;
use std::{any::Any, ops::Neg};
use thiserror::Error;

impl Clone for Value {
    fn clone(&self) -> Value {
        Value(self.0.clone_any())
    }
}

#[derive(Debug, Error)]
pub enum ValueError {
    #[error("negation operation not defined for type `{0}`")]
    OperationNotDefined(&'static str),
}

pub struct Value(Box<dyn CloneAny>);

impl Value {
    pub fn new<T>(v: T) -> Self
    where
        T: CloneAny,
    {
        Value(Box::new(v))
    }
    pub fn get_type_name(&self) -> &'static str {
        self.0.type_name()
    }

    /// Take the negative of the Value if the operation is defined
    pub fn neg(&self) -> Result<Value, NadirParserErrors> {
        if let Some(&val) = self.0.downcast_ref::<f64>() {
            Ok(Value::new(-val))
        } else if let Some(&val) = self.0.downcast_ref::<i64>() {
            Ok(Value::new(-val))
        } else {
            Err(NadirParserErrors::OperationNotDefined)
        }
    }
}
