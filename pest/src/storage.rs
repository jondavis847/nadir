use super::value::Value;
use std::collections::HashMap;
use thiserror::Error;

#[derive(Error, Debug)]
pub enum StorageErrors {
    #[error("variable {0} not found")]
    VariableNotFound(String),
}

#[derive(Default)]
pub struct Storage(HashMap<String, Value>);

impl Storage {
    pub fn insert(&mut self, name: String, value: Value) -> Result<(), StorageErrors> {
        self.0.insert(name, value);
        Ok(())
    }

    pub fn get(&self, name: &str) -> Result<Value, StorageErrors> {
        if let Some(value) = self.0.get(name) {
            Ok(value.clone())
        } else {
            Err(StorageErrors::VariableNotFound(name.to_string()))
        }
    }

    pub fn get_names(&self) -> Vec<&String> {
        let mut names: Vec<&String> = self.0.keys().collect();
        names.sort();
        names
    }
}
