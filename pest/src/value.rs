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
    #[error("cannot promote types {0} and {1}")]
    CannotPromoteTypes(String, String),
}

#[allow(non_camel_case_types)]
pub enum Value {
    f32(f32),
    f64(f64),
    i32(i32),
    i64(i64),
    u32(u32),
    u64(u64),
    Vector3f64(Box<Vector3<f64>>),
    Matrix3f64(Box<Matrix3<f64>>),
    DVector(Box<DVector<f64>>),
    DMatrix(Box<DMatrix<f64>>),
    String(Box<String>),
}

impl Value {
    pub fn as_str(&self) -> String {
        match self {
            Value::f32(_) => String::from("f32"),
            Value::f64(_) => String::from("f64"),
            Value::i32(_) => String::from("i32"),
            Value::i64(_) => String::from("i64"),
            Value::u32(_) => String::from("u32"),
            Value::u64(_) => String::from("u64"),
            Value::Vector3f64(_) => String::from("Vector<f64,3>"),
            Value::Matrix3f64(_) => String::from("Matrix<f64,3,3>"),
            Value::DVector(v) => {
                let length = v.len();
                String::from(format!("Vector<f64,{}>", length))
            }
            Value::DMatrix(v) => {
                let rows = v.nrows();
                let cols = v.ncols();
                String::from(format!("Vector<f64,{},{}>", rows, cols))
            }
            Value::String(_) => String::from("String"),
        }
    }
    pub fn try_add(&self, other: &Value) -> Result<Value, ValueErrors> {
      match try_promote(self, other) {
        Ok(lhs,rhs) => {

        }
        Err(e) => match e {
            ValueErrors::CannotPromoteTypes(, )
        }
      }

            _ => Err(ValueErrors::CannotAddTypes(other.as_str(), self.as_str())),
        }
    }
}

fn try_promote(a: &Value, b: &Value) -> Result<(Value, Value), ValueErrors> {
    match (a, b) {
        (Value::f32(a), Value::f32(b)) => Ok((Value::f32(*a), Value::f32(*b))),
        (Value::f64(a), Value::f32(b)) => Ok((Value::f64(*a), Value::f64(*b as f64))),
        (Value::f32(a), Value::f64(b)) => Ok((Value::f64(*a as f64), Value::f64(*b))),
        (Value::f64(a), Value::f64(b)) => Ok((Value::f64(*a), Value::f64(*b))),
        (Value::i32(a), Value::i32(b)) => Ok((Value::i32(*a), Value::i32(*b))),
        (Value::i64(a), Value::i32(b)) => Ok((Value::i64(*a), Value::i64(*b as i64))),
        (Value::i32(a), Value::i64(b)) => Ok((Value::i64(*a as i64), Value::i64(*b))),
        (Value::i64(a), Value::i64(b)) => Ok((Value::i64(*a), Value::i64(*b))),
        (Value::u32(a), Value::u32(b)) => Ok((Value::u32(*a), Value::u32(*b))),
        (Value::u64(a), Value::u32(b)) => Ok((Value::u64(*a), Value::u64(*b as u64))),
        (Value::u32(a), Value::u64(b)) => Ok((Value::u64(*a as u64), Value::u64(*b))),
        (Value::u64(a), Value::u64(b)) => Ok((Value::u64(*a), Value::u64(*b))),
        (Value::f64(a), Value::i32(b)) => Ok((Value::f64(*a), Value::f64(*b as f64))),
        (Value::f64(a), Value::i64(b)) => Ok((Value::f64(*a), Value::f64(*b as f64))),
        (Value::f64(a), Value::u32(b)) => Ok((Value::f64(*a), Value::f64(*b as f64))),
        (Value::f64(a), Value::u64(b)) => Ok((Value::f64(*a), Value::f64(*b as f64))),
        (Value::f32(a), Value::i32(b)) => Ok((Value::f64(*a as f64), Value::f64(*b as f64))),
        (Value::f32(a), Value::i64(b)) => Ok((Value::f64(*a as f64), Value::f64(*b as f64))),
        (Value::f32(a), Value::u32(b)) => Ok((Value::f64(*a as f64), Value::f64(*b as f64))),
        (Value::f32(a), Value::u64(b)) => Ok((Value::f64(*a as f64), Value::f64(*b as f64))),
        // Add similar rules for Vector3<f64> and Matrix3<f64> if they have a promotion behavior.
        _ => Err(ValueErrors::CannotPromoteTypes(a.as_str(), b.as_str())),
    }
}
