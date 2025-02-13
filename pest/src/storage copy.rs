use std::collections::HashMap;
use thiserror::Error;

/// I am making the masochistic choice to define the storage types explicitly
/// Alternatively this could be done using Box<dyn Any>
/// There is a likely negligible performance hit for using Box<dyn Any>
/// and it reduces lots of boiler plate code, but it also bleeds into all of
/// the other crates as I have to define the types as Any.
/// We don't have that many types, so for now I just define
/// things explicitly.

#[derive(Error, Debug)]
enum StorageError {
    #[error("id {0} not found")]
    IdNotFound(u32),
    #[error("variable {0} not found")]
    VariableNotFound(String),
}

#[derive(Debug, Clone, Copy)]
enum SupportedTypes {
    F64,
    I64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Hash)]
struct Id(u32);

#[derive(Debug, Default)]
struct Identifier {
    next_id: u32,
}
impl Identifier {
    pub fn get_id(&mut self) -> Id {
        let id = Id(self.next_id + 1);
        self.next_id += 1;
        id
    }
}

#[derive(Debug, Clone, Copy)]
pub struct StorageElement {
    id: Id,
    type_id: SupportedTypes,
}

#[derive(Default)]
pub struct Storage {
    identifier: Identifier,
    names: HashMap<String, StorageElement>,
    f64s: HashMap<Id, f64>,
    i64s: HashMap<Id, i64>,
}

impl Storage {
    pub fn add_f64(&mut self, name: String, x: f64) -> Result<(), StorageError> {
        let id = self.identifier.get_id();
        let element = StorageElement {
            id,
            type_id: SupportedTypes::F64,
        };
        self.names.insert(name, element);
        self.f64s.insert(id, x);
        Ok(())
    }

    pub fn add_i64(&mut self, name: String, x: i64) -> Result<(), StorageError> {
        let id = self.identifier.get_id();
        let element = StorageElement {
            id,
            type_id: SupportedTypes::I64,
        };
        self.names.insert(name, element);
        self.i64s.insert(id, x);
        Ok(())
    }

    pub fn get_element(&self, name: &str) -> Result<StorageElement, StorageError> {
        if let Some(element) = self.names.get(name) {
            Ok(*element)
        } else {
            Err(StorageError::VariableNotFound(name.to_string()))
        }
    }

    pub fn get_f64(&self, id: Id) -> Result<f64, StorageError> {
        if let Some(x) = self.f64s.get(&id) {
            Ok(*x)
        } else {
            Err(StorageError::IdNotFound(id.0))
        }
    }

    pub fn get_names(&self) -> Vec<&String> {
        let mut names: Vec<&String> = self.names.keys().collect();
        names.sort();
        names
    }
}
