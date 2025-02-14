use nalgebra::{DMatrix, DVector};
use std::f64::{INFINITY, NAN};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ValueErrors {
    #[error("cannot add type {1} to {0}")]
    CannotAddTypes(String, String),
    #[error("cannot divide type {1} to {0}")]
    CannotDivideTypes(String, String),
    #[error("cannot multiply type {1} to {0}")]
    CannotMultiplyTypes(String, String),
    #[error("cannot calculate negative of type {0}")]
    CannotNegType(String),
    #[error("cannot calculate factorial of type {0}")]
    CannotFactorialType(String),
    #[error("cannot take exponent of type {0}")]
    CannotPowType(String),
    #[error("cannot add type {1} to {0}")]
    CannotSubtractTypes(String, String),
    #[error("cannot calculate factorial of negative number")]
    NegativeFactorial,
    #[error("cannot calculate factorial of a non-integer")]
    NonIntegerFactorial,
    #[error("dimension mismatch: matrix cols {0}, vector rows {1}")]
    SizeMismatch(String, String),
}

#[derive(Clone)]
#[allow(non_camel_case_types)]
pub enum Value {
    f64(f64),
    i64(i64),
    DVector(Box<DVector<f64>>),
    DMatrix(Box<DMatrix<f64>>),
    //String(Box<String>),
}

impl std::fmt::Debug for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Value::f64(v) => writeln!(f, "\x1b[90mf64\x1b[0m {}", v),
            Value::i64(v) => writeln!(f, "\x1b[90mi64\x1b[0m {}", v),
            Value::DVector(v) => {
                writeln!(f, "\x1b[90mVector<f64>\x1b[0m [")?;
                for e in v.iter() {
                    writeln!(f, "     {}", e)?;
                }
                writeln!(f, "]")
            }
            Value::DMatrix(m) => {
                writeln!(
                    f,
                    "\x1b[90mMatrix<f64>\x1b[0m ({}x{})",
                    m.nrows(),
                    m.ncols()
                )?;
                writeln!(f, "[")?;
                for row in m.row_iter() {
                    writeln!(f, "  {:?}", row)?;
                }
                writeln!(f, "]")
            } //Value::String(s) => writeln!(f, "\x1b[90mString\x1b[0m {}", s),
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
            } //Value::String(_) => String::from("String"),
        }
    }
    pub fn try_add(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::f64(a + b)),
            (Value::i64(a), Value::f64(b)) | (Value::f64(b), Value::i64(a)) => {
                Ok(Value::f64(*a as f64 + b))
            }
            (Value::i64(a), Value::i64(b)) => Ok(Value::i64(a + b)),
            (Value::f64(a), Value::DVector(v)) | (Value::DVector(v), Value::f64(a)) => {
                Ok(Value::DVector(Box::new(v.add_scalar(*a))))
            }
            (Value::DMatrix(m), Value::f64(f)) | (Value::f64(f), Value::DMatrix(m)) => {
                Ok(Value::DMatrix(Box::new(m.add_scalar(*f))))
            }
            (Value::i64(a), Value::DVector(v)) | (Value::DVector(v), Value::i64(a)) => {
                Ok(Value::DVector(Box::new(v.add_scalar(*a as f64))))
            }
            (Value::DMatrix(m), Value::i64(f)) | (Value::i64(f), Value::DMatrix(m)) => {
                Ok(Value::DMatrix(Box::new(m.add_scalar(*f as f64))))
            }
            // (Value::String(a), Value::String(b)) => {
            //     Ok(Value::String(Box::new(format!("{}{}", a, b))))
            // }
            _ => Err(ValueErrors::CannotAddTypes(other.as_str(), self.as_str())),
        }
    }

    pub fn try_sub(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::f64(a - b)),
            (Value::i64(a), Value::f64(b)) => Ok(Value::f64(*a as f64 - b)),
            (Value::f64(a), Value::i64(b)) => Ok(Value::f64(a - *b as f64)),
            (Value::i64(a), Value::i64(b)) => Ok(Value::i64(a - b)),
            (Value::DVector(v), Value::f64(a)) => Ok(Value::DVector(Box::new(v.add_scalar(-*a)))),
            (Value::DVector(v), Value::i64(a)) => {
                Ok(Value::DVector(Box::new(v.add_scalar(-(*a as f64)))))
            }
            (Value::DMatrix(m), Value::f64(f)) => Ok(Value::DMatrix(Box::new(m.add_scalar(-*f)))),
            (Value::DMatrix(m), Value::i64(f)) => {
                Ok(Value::DMatrix(Box::new(m.add_scalar(-(*f as f64)))))
            }
            _ => Err(ValueErrors::CannotSubtractTypes(
                other.as_str(),
                self.as_str(),
            )),
        }
    }

    pub fn try_mul(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::f64(a * b)),
            (Value::i64(a), Value::f64(b)) | (Value::f64(b), Value::i64(a)) => {
                Ok(Value::f64(*a as f64 * b))
            }
            (Value::i64(a), Value::i64(b)) => Ok(Value::i64(a * b)),
            (Value::f64(a), Value::DVector(v)) | (Value::DVector(v), Value::f64(a)) => {
                Ok(Value::DVector(Box::new(v.scale(*a))))
            }
            (Value::DMatrix(m), Value::f64(f)) | (Value::f64(f), Value::DMatrix(m)) => {
                Ok(Value::DMatrix(Box::new(m.scale(*f))))
            }
            (Value::i64(a), Value::DVector(v)) | (Value::DVector(v), Value::i64(a)) => {
                Ok(Value::DVector(Box::new(v.scale(*a as f64))))
            }
            (Value::DMatrix(m), Value::i64(f)) | (Value::i64(f), Value::DMatrix(m)) => {
                Ok(Value::DMatrix(Box::new(m.scale(*f as f64))))
            }
            (Value::DMatrix(a), Value::DVector(b)) => {
                // Dereference the Box to get the underlying types
                let a: &DMatrix<f64> = &**a;
                let b: &DVector<f64> = &**b;

                // Check that the matrix and vector dimensions are compatible.
                // (For a multiplication a * b to work, the number of columns of 'a' must equal the number of rows of 'b',
                // which is equivalent to b.len() for a column vector.)
                if a.ncols() == b.len() {
                    // If nalgebra supports matrix * vector multiplication directly, you can simply do:
                    let result: DVector<f64> = a * b;
                    // Wrap the resulting DVector in a Box if needed
                    Ok(Value::DVector(Box::new(result)))
                } else {
                    Err(ValueErrors::SizeMismatch(
                        a.ncols().to_string(),
                        b.len().to_string(),
                    ))
                }
            }
            (Value::DMatrix(a), Value::DMatrix(b)) => {
                Ok(Value::DMatrix(Box::new(*a.clone() * *b.clone())))
            }
            _ => Err(ValueErrors::CannotMultiplyTypes(
                other.as_str(),
                self.as_str(),
            )),
        }
    }

    pub fn try_div(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => {
                Ok(Value::f64(a / b)) // This will return inf or NaN for division by zero
            }
            (Value::i64(a), Value::f64(b)) => {
                Ok(Value::f64(*a as f64 / b)) // This will return inf or NaN for division by zero
            }
            (Value::f64(a), Value::i64(b)) => {
                if *b == 0 {
                    if *a == 0.0 {
                        Ok(Value::f64(NAN))
                    } else if *a > 0.0 {
                        Ok(Value::f64(INFINITY))
                    } else {
                        Ok(Value::f64(-INFINITY))
                    }
                } else {
                    Ok(Value::f64(a / *b as f64))
                }
            }
            (Value::i64(a), Value::i64(b)) => {
                if *b == 0 {
                    if *a == 0 {
                        Ok(Value::f64(NAN))
                    } else if *a > 0 {
                        Ok(Value::f64(INFINITY))
                    } else {
                        Ok(Value::f64(-INFINITY))
                    }
                } else {
                    Ok(Value::i64(a / b))
                }
            }
            (Value::DVector(v), Value::f64(a)) => {
                Ok(Value::DVector(Box::new(v.scale(1.0 / *a)))) // This will produce inf or NaN elements for division by zero
            }
            (Value::DMatrix(m), Value::f64(f)) => {
                Ok(Value::DMatrix(Box::new(m.scale(1.0 / *f)))) // This will produce inf or NaN elements for division by zero
            }
            (Value::DVector(v), Value::i64(a)) => {
                if *a == 0 {
                    Ok(Value::DVector(Box::new(v.map(|x| {
                        if x == 0.0 {
                            NAN
                        } else if x > 0.0 {
                            INFINITY
                        } else {
                            -INFINITY
                        }
                    }))))
                } else {
                    Ok(Value::DVector(Box::new(v.scale(1.0 / *a as f64))))
                }
            }
            (Value::DMatrix(m), Value::i64(f)) => {
                if *f == 0 {
                    Ok(Value::DMatrix(Box::new(m.map(|x| {
                        if x == 0.0 {
                            NAN
                        } else if x > 0.0 {
                            INFINITY
                        } else {
                            -INFINITY
                        }
                    }))))
                } else {
                    Ok(Value::DMatrix(Box::new(m.scale(1.0 / *f as f64))))
                }
            }
            _ => Err(ValueErrors::CannotDivideTypes(
                other.as_str(),
                self.as_str(),
            )),
        }
    }

    pub fn try_negative(&self) -> Result<Value, ValueErrors> {
        match self {
            Value::f64(v) => Ok(Value::f64(-v)),
            Value::i64(v) => Ok(Value::i64(-v)),
            Value::DVector(v) => Ok(Value::DVector(Box::new(-*v.clone()))),
            Value::DMatrix(v) => Ok(Value::DMatrix(Box::new(-*v.clone()))),
            //_ => Err(ValueErrors::CannotNegType(self.as_str())),
        }
    }

    pub fn try_factorial(&self) -> Result<Value, ValueErrors> {
        match self {
            Value::i64(v) if *v >= 0 => {
                let result = (1..=*v).product::<i64>();
                Ok(Value::i64(result))
            }
            Value::i64(_) => Err(ValueErrors::NegativeFactorial),
            Value::f64(v) if *v >= 0.0 && v.fract() == 0.0 => {
                let result = (1..=(v.round() as u64)).product::<u64>();
                Ok(Value::f64(result as f64))
            }
            Value::f64(v) if *v < 0.0 => Err(ValueErrors::NegativeFactorial),
            Value::f64(_) => Err(ValueErrors::NonIntegerFactorial),
            _ => Err(ValueErrors::CannotFactorialType(self.as_str())),
        }
    }

    pub fn try_pow(&self, exponent: &Value) -> Result<Value, ValueErrors> {
        match (self, exponent) {
            (Value::i64(base), Value::i64(exp)) => {
                if *exp >= 0 {
                    Ok(Value::i64(base.pow(*exp as u32)))
                } else {
                    Ok(Value::f64((base.pow(-*exp as u32) as f64).recip()))
                }
            }
            (Value::f64(base), Value::i64(exp)) => Ok(Value::f64(base.powi(*exp as i32))),
            (Value::i64(base), Value::f64(exp)) => Ok(Value::f64((*base as f64).powf(*exp))),
            (Value::f64(base), Value::f64(exp)) => Ok(Value::f64(base.powf(*exp))),
            _ => Err(ValueErrors::CannotPowType(self.as_str())),
        }
    }
}
