use crate::{Rule, registry::RegistryErrors, storage::StorageErrors, value::ValueErrors};
use ansi_term::Colour;
use std::fmt;
use time::TimeErrors;

#[derive(Debug)]
pub enum ReplErrors {
    CantParseMatrix,
    EmptyValue,
    InvalidEnum(String, String),
    InvalidField(String, String, String),
    MatrixRowLengthMismatch,
    NameReserved(&'static str),
    RegistryErrors(RegistryErrors),
    OutOfBoundsIndex(String, String),
    StorageErrors(StorageErrors),
    TimeErrors(TimeErrors),
    UnexpectedRule(Rule),
    ValueErrors(ValueErrors),
}

impl From<RegistryErrors> for ReplErrors {
    fn from(value: RegistryErrors) -> Self {
        Self::RegistryErrors(value)
    }
}

impl From<StorageErrors> for ReplErrors {
    fn from(value: StorageErrors) -> Self {
        Self::StorageErrors(value)
    }
}

impl From<ValueErrors> for ReplErrors {
    fn from(value: ValueErrors) -> Self {
        Self::ValueErrors(value)
    }
}

impl From<TimeErrors> for ReplErrors {
    fn from(value: TimeErrors) -> Self {
        Self::TimeErrors(value)
    }
}

impl fmt::Display for ReplErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let error_message = match self {
            ReplErrors::CantParseMatrix => "could not parse matrix".to_string(),
            ReplErrors::EmptyValue => format!("pest parsing value was empty"),
            ReplErrors::InvalidEnum(name, variant) => {
                format!("invalid enum {name}::{variant}")
            }
            ReplErrors::InvalidField(instance, field, type_name) => {
                format!("invalid field {field} for variable {instance} of type {type_name}")
            }
            ReplErrors::MatrixRowLengthMismatch => {
                format!("matrix cannot have rows of different lengths")
            }
            ReplErrors::NameReserved(str) => format!(
                "variable name '{}' is reserverd",
                str
            ),
            ReplErrors::OutOfBoundsIndex(max, ind) => {
                format!("max size was {max} but index was {ind}")
            }
            ReplErrors::RegistryErrors(err) => format!("{}", err),
            ReplErrors::StorageErrors(err) => format!("{}", err),
            ReplErrors::TimeErrors(err) => format!("{}", err),
            ReplErrors::UnexpectedRule(rule) => format!("unexpected rule: {:?}", rule),
            ReplErrors::ValueErrors(err) => format!("{}", err),
        };

        // Wrap the error message in red
        let error_message = Colour::Red.paint(error_message);
        write!(f, "{}", error_message)
    }
}
