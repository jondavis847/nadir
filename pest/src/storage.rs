use super::clone_any::CloneAny;
use super::value::Value;
use std::collections::HashMap;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum StorageError {
    #[error("variable {0} not found")]
    VariableNotFound(String),
}

#[derive(Default)]
pub struct Storage(HashMap<String, Value>);

impl Storage {
    pub fn insert(&mut self, name: String, x: &dyn CloneAny) -> Result<(), StorageError> {
        self.0.insert(name, Value::new(x.clone_any()));
        Ok(())
    }

    pub fn get(&self, name: &str) -> Result<Value, StorageError> {
        if let Some(value) = self.0.get(name) {
            Ok(value.clone())
        } else {
            Err(StorageError::VariableNotFound(name.to_string()))
        }
    }

    pub fn get_names(&self) -> Vec<&String> {
        let mut names: Vec<&String> = self.0.keys().collect();
        names.sort();
        names
    }
}
