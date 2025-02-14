use nalgebra::{DMatrix, DVector, Matrix3, Vector3};
use thiserror::Error;

use crate::NadirParserErrors;

// use super::clone_any::CloneAny;
// use std::{any::Any, ops::Neg};
// use thiserror::Error;

// impl Clone for Value {
//     fn clone(&self) -> Value {
//         Value(self.0.clone_any())
//     }
// }

// #[derive(Debug, Error)]
// pub enum ValueError {
//     #[error("negation operation not defined for type `{0}`")]
//     OperationNotDefined(&'static str),
// }

// pub struct Value(Box<dyn CloneAny>);

// impl Value {
//     pub fn new<T>(v: T) -> Self
//     where
//         T: CloneAny,
//     {
//         Value(Box::new(v))
//     }
//     pub fn get_type_name(&self) -> &'static str {
//         self.0.type_name()
//     }

//     /// Take the negative of the Value if the operation is defined
//     pub fn neg(&self) -> Result<Value, NadirParserErrors> {
//         if let Some(&val) = self.0.downcast_ref::<f64>() {
//             Ok(Value::new(-val))
//         } else if let Some(&val) = self.0.downcast_ref::<i64>() {
//             Ok(Value::new(-val))
//         } else {
//             Err(NadirParserErrors::OperationNotDefined)
//         }
//     }
// }
#[derive(Debug, Error)]
pub enum ValueErrors {
    #[error("cannot add type {1} to {0}")]
    CannotAddTypes(String, String),
    #[error("cannot take negative of type {0}")]
    CannotNegType(String),
}

#[derive(Clone)]
#[allow(non_camel_case_types)]
pub enum Value {
    f64(f64),
    i64(i64),
    DVector(Box<DVector<f64>>),
    DMatrix(Box<DMatrix<f64>>),
    String(Box<String>),
}

impl std::fmt::Debug for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Value::f64(v) => writeln!(f, "f64: {}", v),
            Value::i64(v) => writeln!(f, "i64: {}", v),
            Value::DVector(v) => {
                writeln!(f, "Vector<f64>: [");
                for e in v.iter() {
                    writeln!(f, "     {}", e);
                }
                writeln!(f, "]")
            }
            Value::DMatrix(m) => {
                writeln!(f, "Matrix<f64>: ({}x{})", m.nrows(), m.ncols())?;
                writeln!(f, "[")?;
                for row in m.row_iter() {
                    writeln!(f, "  {:?}", row)?;
                }
                writeln!(f, "]")
            }
            Value::String(s) => writeln!(f, "String: {}", s),
        }
    }
}

impl Value {
    pub fn as_str(&self) -> String {
        match self {
            Value::f64(_) => String::from("f64"),
            Value::i64(_) => String::from("i64"),
            Value::DVector(v) => {
                let length = v.len();
                String::from(format!("Vector<f64,{}>", length))
            }
            Value::DMatrix(v) => {
                let rows = v.nrows();
                let cols = v.ncols();
                String::from(format!("Matrix<f64,{},{}>", rows, cols))
            }
            Value::String(_) => String::from("String"),
        }
    }
    pub fn try_add(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::f64(a + b)),
            (Value::i64(a), Value::f64(b)) => Ok(Value::f64(*a as f64 + b)),
            (Value::f64(a), Value::i64(b)) => Ok(Value::f64(a + *b as f64)),
            (Value::i64(a), Value::i64(b)) => Ok(Value::i64(a + b)),
            (Value::f64(a), Value::DVector(v)) => Ok(Value::DVector(Box::new(v.add_scalar(*a)))),
            (Value::DVector(v), Value::f64(a)) => Ok(Value::DVector(Box::new(v.add_scalar(*a)))),
            (Value::DMatrix(m), Value::f64(f)) => Ok(Value::DMatrix(Box::new(m.add_scalar(*f)))),
            (Value::f64(f), Value::DMatrix(m)) => Ok(Value::DMatrix(Box::new(m.add_scalar(*f)))),
            (Value::i64(a), Value::DVector(v)) => {
                Ok(Value::DVector(Box::new(v.add_scalar(*a as f64))))
            }
            (Value::DVector(v), Value::i64(a)) => {
                Ok(Value::DVector(Box::new(v.add_scalar(*a as f64))))
            }
            (Value::DMatrix(m), Value::i64(f)) => {
                Ok(Value::DMatrix(Box::new(m.add_scalar(*f as f64))))
            }
            (Value::i64(f), Value::DMatrix(m)) => {
                Ok(Value::DMatrix(Box::new(m.add_scalar(*f as f64))))
            }
            (Value::String(a), Value::String(b)) => {
                Ok(Value::String(Box::new(String::from(format!("{}{}", a, b)))))
            }
            _ => Err(ValueErrors::CannotAddTypes(other.as_str(), self.as_str())),
        }
    }

    pub fn try_neg(&self) -> Result<Value, ValueErrors> {
        match self {
            Value::f64(v) => Ok(Value::f64(-v)),
            Value::i64(v) => Ok(Value::i64(-v)),
            Value::DVector(v) => Ok(Value::DVector(Box::new(-*v.clone()))),
            Value::DMatrix(v) => Ok(Value::DMatrix(Box::new(-*v.clone()))),
            _ => Err(ValueErrors::CannotNegType(self.as_str())),
        }
    }
}
