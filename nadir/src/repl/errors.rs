use crate::{Rule, registry::RegistryErrors, storage::StorageErrors, value::ValueErrors};
use ansi_term::Colour;
use std::fmt;
use time::TimeErrors;

#[derive(Debug)]
pub enum ReplErrors {
    ArgumentValueIsNone(String),
    CantParseVector,
    CantParseMatrix,
    EmptyPairs(String),
    EmptyValue,
    ExpectedFunctionArguments(String),
    ExpectedFunctionCall(String),
    ExpectedIdentifier(String),
    FunctionNotFound(String),
    InvalidEnum(String, String),
    InvalidField(String, String, String),
    InvalidMethod(String, String, String),
    InvalidNumber(String),
    MatrixRowLengthMismatch,
    MissingFunctionArguments,
    MissingFunctionName,
    NameReserved(&'static str),
    NumberOfArgs(String, String, String),
    RegistryErrors(RegistryErrors),
    OutOfBoundsIndex(String, String),
    StorageErrors(StorageErrors),
    TimeErrors(TimeErrors),
    UnexpectedOperator(String),
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
            ReplErrors::ArgumentValueIsNone(arg) => {
                format!("argument value was None: {}", arg)
            }
            ReplErrors::CantParseVector => "could not parse vector".to_string(),
            ReplErrors::CantParseMatrix => "could not parse matrix".to_string(),
            ReplErrors::EmptyValue => format!("pest parsing value was empty"),
            ReplErrors::EmptyPairs(arg) => format!("pest parsing pairs were empty: {}", arg),
            ReplErrors::ExpectedFunctionArguments(arg) => {
                format!("expected rule was function_arguments but got: {}", arg)
            }
            ReplErrors::ExpectedFunctionCall(arg) => {
                format!("expected rule was function_call but got: {}", arg)
            }
            ReplErrors::ExpectedIdentifier(arg) => {
                format!("expected rule was identifier but got: {}", arg)
            }
            ReplErrors::FunctionNotFound(func) => format!("function not found: {}", func),
            ReplErrors::InvalidMethod(instance, method, type_name) => {
                format!("invalid method {method} for variable {instance} of type {type_name}")
            }
            ReplErrors::InvalidEnum(name, variant) => {
                format!("invalid enum {name}::{variant}")
            }
            ReplErrors::InvalidField(instance, field, type_name) => {
                format!("invalid field {field} for variable {instance} of type {type_name}")
            }
            ReplErrors::InvalidNumber(num) => format!("invalid number: {}", num),
            ReplErrors::MatrixRowLengthMismatch => {
                format!("matrix cannot have rows of different lengths")
            }
            ReplErrors::MissingFunctionArguments => {
                "was not able to parse function arguments".to_string()
            }
            ReplErrors::MissingFunctionName => "was not able to parse a function name".to_string(),
            ReplErrors::NameReserved(str) => format!("variable name '{}' is reserverd", str),
            ReplErrors::NumberOfArgs(func, expected, got) => {
                format!(
                    "incorrect number of args to function '{}'. expected {}, got {}",
                    func, expected, got
                )
            }
            ReplErrors::OutOfBoundsIndex(max, ind) => {
                format!("max size was {max} but index was {ind}")
            }
            ReplErrors::RegistryErrors(err) => format!("{}", err),
            ReplErrors::StorageErrors(err) => format!("{}", err),
            ReplErrors::TimeErrors(err) => format!("{}", err),
            ReplErrors::UnexpectedOperator(op) => format!("unexpected operator: {}", op),
            ReplErrors::UnexpectedRule(rule) => format!("unexpected rule: {:?}", rule),
            ReplErrors::ValueErrors(err) => format!("{}", err),
        };

        // Wrap the error message in red
        let error_message = Colour::Red.paint(error_message);
        write!(f, "{}", error_message)
    }
}
