use nalgebra::{DMatrix, DVector};
use rotations::{
    //euler_angles::{EulerAngles, EulerSequence},
    quaternion::{Quaternion, UnitQuaternion},
    //rotation_matrix::RotationMatrix,
};
use unicode_width::UnicodeWidthStr;

use std::f64::{INFINITY, NAN};
use thiserror::Error;
use time::{Time, TimeSystem};

pub fn label(s: &str) -> String {
    ansi_term::Colour::Fixed(237).paint(s).to_string()
}

#[derive(Debug, Error)]
pub enum ValueErrors {
    #[error("cannot add type {1} to {0}")]
    CannotAddTypes(String, String),
    #[error("cannot compare type {1} by {0}")]
    CannotCompareTypes(String, String),
    #[error("{0}")]
    CannotConvertToBool(String),
    #[error("cannot convert type {0} to f64")]
    CannotConvertToF64(String),
    #[error("cannot convert type {0} to i64")]
    CannotConvertToI64(String),
    #[error("cannot convert type {0} to Quaternion")]
    CannotConvertToQuaternion(String),
    #[error("cannot convert type {0} to UnitQuaternion")]
    CannotConvertToUnitQuaternion(String),
    #[error("cannot convert type {0} to usize")]
    CannotConvertToUsize(String),
    #[error("cannot divide type {1} by {0}")]
    CannotDivideTypes(String, String),
    #[error("cannot index type {0}")]
    CannotIndexType(String),
    #[error("cannot multiply type {1} by {0}")]
    CannotMultiplyTypes(String, String),
    #[error("cannot calculate negative of type {0}")]
    CannotNegType(String),
    #[error("cannot calculate factorial of type {0}")]
    CannotFactorialType(String),
    #[error("cannot calculate exponent of type {0}")]
    CannotPowType(String),
    #[error("cannot add type {1} to {0}")]
    CannotSubtractTypes(String, String),
    #[error("index was {0}, can't be negative")]
    NegativeIndex(String),
    #[error("cannot calculate factorial of negative number")]
    NegativeFactorial,
    #[error("cannot index with type {0}")]
    NonIndexType(String),
    #[error("cannot calculate factorial of a non-integer")]
    NonIntegerFactorial,
    #[error("out of bounds index. got {0}, max is {1}")]
    OutOfBoundsIndex(usize, usize),
    #[error("dimension mismatch: matrix cols {0}, vector rows {1}")]
    SizeMismatch(String, String),
}

#[derive(Clone)]
#[allow(non_camel_case_types)]
pub enum Value {
    f64(f64),
    i64(i64),
    bool(bool),
    Enum(Enum),
    Vector(Box<DVector<f64>>),
    VectorBool(Box<DVector<bool>>),
    VectorUsize(Box<DVector<usize>>),
    Matrix(Box<DMatrix<f64>>),
    None,
    Range(Range),
    Quaternion(Box<Quaternion>),
    UnitQuaternion(Box<UnitQuaternion>),
    String(Box<String>),
    Time(Box<Time>),
}

impl std::fmt::Debug for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Value::f64(v) => {
                if v.fract().abs() < std::f64::EPSILON {
                    // Print with one decimal place (appending ".0")
                    writeln!(f, "{} {:.1}", label("f64"), v)
                } else {
                    // Print the full decimal representation
                    writeln!(f, "{} {}", label("f64"), v)
                }
            }
            Value::i64(v) => writeln!(f, "{} {}", label("i64"), v),
            Value::bool(v) => writeln!(f, "{} {}", label("bool"), v),
            Value::Enum(e) => writeln!(f, "{}::{}", e.name, e.variant),
            Value::None => writeln!(f, "{}", label("None")),
            Value::Range(r) => {
                writeln!(f, "{}", label("Range"))?;
                let start = if let Some(start) = r.start {
                    start.to_string()
                } else {
                    "None".to_string()
                };
                let stop = if let Some(stop) = r.stop {
                    stop.to_string()
                } else {
                    "None".to_string()
                };
                let step = if let Some(step) = r.step {
                    step.to_string()
                } else {
                    "None".to_string()
                };
                writeln!(f, "start: {}", start)?;
                writeln!(f, "stop: {}", stop)?;
                writeln!(f, "step: {}", step)
            }
            Value::Vector(v) => {
                writeln!(f, "{}", label(&format!("Vector<f64,{}>", v.len())))?;
                for (i, e) in v.iter().enumerate() {
                    // Check if the fractional part is effectively 0
                    if e.fract().abs() < std::f64::EPSILON {
                        // Print with one decimal place (appending ".0")
                        writeln!(f, "{} {:.1}", label(&i.to_string()), e)?;
                    } else {
                        // Print the full decimal representation
                        writeln!(f, "{} {}", label(&i.to_string()), e)?;
                    }
                }
                Ok(())
            }
            Value::VectorUsize(v) => {
                writeln!(f, "{}", label(&format!("Vector<usize,{}>", v.len())))?;
                for (i, e) in v.iter().enumerate() {
                    writeln!(f, "{} {}", label(&i.to_string()), e)?;
                }
                Ok(())
            }
            Value::VectorBool(v) => {
                writeln!(f, "{}", label(&format!("Vector<bool,{}>", v.len())))?;
                for (i, e) in v.iter().enumerate() {
                    writeln!(f, "{} {}", label(&i.to_string()), e)?;
                }
                Ok(())
            }
            Value::Matrix(m) => {
                writeln!(
                    f,
                    "{}",
                    label(&format!("Matrix<f64,{}x{}>", m.nrows(), m.ncols()))
                )?;

                let col_width: usize = 8; // Explicitly define col_width as `usize`

                // Print column headers (aligned with columns)
                write!(f, "         ")?; // Space for row index alignment
                for j in 0..m.ncols() {
                    let header = label(&j.to_string()); // Apply color formatting
                    let header_width: usize = UnicodeWidthStr::width(j.to_string().as_str()); // Get actual width
                    let padding: usize = col_width.saturating_sub(header_width); // Explicitly declare as usize

                    write!(f, "{}{:width$} ", header, "", width = padding)?; // Corrected alignment
                }
                writeln!(f)?; // New line after header

                // Print matrix rows with row indices
                for (i, row) in m.row_iter().enumerate() {
                    write!(f, "{:>3} ", label(&i.to_string()))?; // Print row index with spacing

                    for &value in row.iter() {
                        if value.fract().abs() < 1e-6 {
                            write!(f, "{:width$.1} ", value, width = col_width)?;
                        } else {
                            write!(f, "{:width$.5} ", value, width = col_width)?;
                        }
                    }
                    writeln!(f)?;
                }
                Ok(())
            }

            Value::String(s) => {
                writeln!(f, "{}", label("String"))?;
                writeln!(f, "{}", s)
            }
            Value::Quaternion(q) => {
                writeln!(f, "{}", label("Quaternion"))?;
                writeln!(f, "{} {}", label("x"), q.x)?;
                writeln!(f, "{} {}", label("y"), q.y)?;
                writeln!(f, "{} {}", label("z"), q.z)?;
                writeln!(f, "{} {}", label("w"), q.w)
            }
            Value::Time(t) => {
                let time_label = match t.system {
                    TimeSystem::GPS => "Time<GPS>",
                    TimeSystem::TAI => "Time<TAI>",
                    TimeSystem::UTC => "Time<UTC>",
                    TimeSystem::TDB => "Time<TDB>",
                    TimeSystem::TT => "Time<TT>",
                };
                // Default to always showing the datetime, and using methods for other values
                let value = t.get_datetime().to_string();

                writeln!(f, "{}", label(time_label))?;
                writeln!(f, "{value}")
            }
            Value::UnitQuaternion(q) => {
                writeln!(f, "{}", label("UnitQuaternion"))?;
                writeln!(f, "{} {}", label("x"), q.0.x)?;
                writeln!(f, "{} {}", label("y"), q.0.y)?;
                writeln!(f, "{} {}", label("z"), q.0.z)?;
                writeln!(f, "{} {}", label("w"), q.0.w)
            } //Value::String(s) => writeln!(f, "\x1b[90mString\x1b[0m {}", s),
        }
    }
}

impl Value {
    pub fn to_string(&self) -> String {
        match self {
            Value::f64(_) => String::from("f64"),
            Value::i64(_) => String::from("i64"),
            Value::bool(_) => String::from("bool"),
            Value::Enum(_) => String::from("Enum"),
            Value::Vector(v) => {
                let length = v.len();
                String::from(format!("Vector<f64,{}>", length))
            }
            Value::VectorUsize(v) => {
                let length = v.len();
                String::from(format!("Vector<usize,{}>", length))
            }
            Value::VectorBool(v) => {
                let length = v.len();
                String::from(format!("Vector<bool,{}>", length))
            }
            Value::Matrix(v) => {
                let rows = v.nrows();
                let cols = v.ncols();
                String::from(format!("Matrix<f64,{},{}>", rows, cols))
            }
            Value::Time(_) => "Time".to_string(),
            Value::None => "None".to_string(),
            Value::Range(_) => String::from("Range"),
            Value::Quaternion(_) => String::from("Quaternion"),
            Value::UnitQuaternion(_) => String::from("UnitQuaternion"),
            Value::String(_) => String::from("String"),
        }
    }

    pub fn as_bool(&self) -> Result<bool, ValueErrors> {
        match self {
            Value::bool(v) => Ok(*v),

            Value::f64(v) => {
                if *v == 0.0 {
                    Ok(false)
                } else if *v == 1.0 {
                    Ok(true)
                } else {
                    Err(ValueErrors::CannotConvertToBool(
                        "f64 must be 0.0 or 1.0 to convert to bool".to_string(),
                    ))
                }
            }

            Value::i64(v) => {
                if *v == 0 {
                    Ok(false)
                } else if *v == 1 {
                    Ok(true)
                } else {
                    Err(ValueErrors::CannotConvertToBool(
                        "i64 must be 0 or 1 to convert to bool".to_string(),
                    ))
                }
            }

            _ => Err(ValueErrors::CannotConvertToBool(format!(
                "cannot convert type {} to bool",
                self.to_string()
            ))),
        }
    }

    pub fn as_f64(&self) -> Result<f64, ValueErrors> {
        match self {
            Value::f64(v) => Ok(*v),
            Value::i64(v) => Ok(*v as f64),
            _ => Err(ValueErrors::CannotConvertToF64(self.to_string())),
        }
    }

    pub fn as_i64(&self) -> Result<i64, ValueErrors> {
        match self {
            Value::f64(v) => Ok(*v as i64),
            Value::i64(v) => Ok(*v),
            _ => Err(ValueErrors::CannotConvertToI64(self.to_string())),
        }
    }

    pub fn as_index(&self) -> Result<IndexStyle, ValueErrors> {
        match self {
            Value::i64(v) => {
                if *v < 0 {
                    return Err(ValueErrors::NegativeIndex(v.to_string()));
                }
                Ok(IndexStyle::Usize(*v as usize))
            }
            Value::VectorBool(v) => Ok(IndexStyle::VecBool(Box::new(*v.clone()))),
            Value::VectorUsize(v) => Ok(IndexStyle::VecUsize(Box::new(*v.clone()))),
            _ => Err(ValueErrors::NonIndexType(self.to_string())),
        }
    }

    pub fn as_usize(&self) -> Result<usize, ValueErrors> {
        match self {
            Value::f64(v) => Ok(*v as usize),
            Value::i64(v) => Ok(*v as usize),
            _ => Err(ValueErrors::CannotConvertToUsize(self.to_string())),
        }
    }

    pub fn as_quaternion(&self) -> Result<Quaternion, ValueErrors> {
        match self {
            Value::Quaternion(v) => Ok(**v),
            _ => Err(ValueErrors::CannotConvertToQuaternion(self.to_string())),
        }
    }

    pub fn as_unit_quaternion(&self) -> Result<UnitQuaternion, ValueErrors> {
        match self {
            Value::UnitQuaternion(v) => Ok(**v),
            _ => Err(ValueErrors::CannotConvertToUnitQuaternion(self.to_string())),
        }
    }

    pub fn try_add(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::f64(a + b)),
            (Value::i64(a), Value::f64(b)) | (Value::f64(b), Value::i64(a)) => {
                Ok(Value::f64(*a as f64 + b))
            }
            (Value::i64(a), Value::i64(b)) => Ok(Value::i64(a + b)),
            (Value::f64(a), Value::Vector(v)) | (Value::Vector(v), Value::f64(a)) => {
                Ok(Value::Vector(Box::new(v.add_scalar(*a))))
            }
            (Value::Matrix(m), Value::f64(f)) | (Value::f64(f), Value::Matrix(m)) => {
                Ok(Value::Matrix(Box::new(m.add_scalar(*f))))
            }
            (Value::i64(a), Value::Vector(v)) | (Value::Vector(v), Value::i64(a)) => {
                Ok(Value::Vector(Box::new(v.add_scalar(*a as f64))))
            }
            (Value::Matrix(m), Value::i64(f)) | (Value::i64(f), Value::Matrix(m)) => {
                Ok(Value::Matrix(Box::new(m.add_scalar(*f as f64))))
            }
            (Value::Vector(v1), Value::Vector(v2)) => {
                if v1.len() != v2.len() {
                    return Err(ValueErrors::SizeMismatch(
                        v1.len().to_string(),
                        v2.len().to_string(),
                    ));
                }
                let mut v3 = *v1.clone();
                for i in 0..v1.len() {
                    v3[i] = v1[i] + v2[i];
                }
                Ok(Value::Vector(Box::new(v3)))
            }
            (Value::Matrix(v1), Value::Matrix(v2)) => {
                if v1.nrows() != v2.nrows() || v1.ncols() != v2.ncols() {
                    return Err(ValueErrors::SizeMismatch(
                        format!("{}x{}", v1.nrows(), v1.ncols()),
                        format!("{}x{}", v2.nrows(), v2.ncols()),
                    ));
                }
                let mut v3 = *v1.clone();
                for i in 0..v1.nrows() {
                    for j in 0..v1.ncols() {
                        v3[(i, j)] = v1[(i, j)] + v2[(i, j)];
                    }
                }
                Ok(Value::Matrix(Box::new(v3)))
            }
            (Value::String(a), Value::String(b)) => {
                Ok(Value::String(Box::new(format!("{}{}", a, b))))
            }
            _ => Err(ValueErrors::CannotAddTypes(
                other.to_string(),
                self.to_string(),
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
            (Value::Vector(v), Value::f64(a)) => {
                Ok(Value::Vector(Box::new(v.scale(1.0 / *a)))) // This will produce inf or NaN elements for division by zero
            }
            (Value::Matrix(m), Value::f64(f)) => {
                Ok(Value::Matrix(Box::new(m.scale(1.0 / *f)))) // This will produce inf or NaN elements for division by zero
            }
            (Value::Vector(v), Value::i64(a)) => {
                if *a == 0 {
                    Ok(Value::Vector(Box::new(v.map(|x| {
                        if x == 0.0 {
                            NAN
                        } else if x > 0.0 {
                            INFINITY
                        } else {
                            -INFINITY
                        }
                    }))))
                } else {
                    Ok(Value::Vector(Box::new(v.scale(1.0 / *a as f64))))
                }
            }
            (Value::Matrix(m), Value::i64(f)) => {
                if *f == 0 {
                    Ok(Value::Matrix(Box::new(m.map(|x| {
                        if x == 0.0 {
                            NAN
                        } else if x > 0.0 {
                            INFINITY
                        } else {
                            -INFINITY
                        }
                    }))))
                } else {
                    Ok(Value::Matrix(Box::new(m.scale(1.0 / *f as f64))))
                }
            }
            _ => Err(ValueErrors::CannotDivideTypes(
                other.to_string(),
                self.to_string(),
            )),
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
            _ => Err(ValueErrors::CannotFactorialType(self.to_string())),
        }
    }

    pub fn try_gt(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::bool(a > b)),
            (Value::i64(a), Value::f64(b)) => Ok(Value::bool(*a as f64 > *b)),
            (Value::f64(a), Value::i64(b)) => Ok(Value::bool(*a > *b as f64)),
            (Value::i64(a), Value::i64(b)) => Ok(Value::bool(a > b)),
            _ => Err(ValueErrors::CannotCompareTypes(
                other.to_string(),
                self.to_string(),
            )),
        }
    }

    pub fn try_gte(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::bool(a >= b)),
            (Value::i64(a), Value::f64(b)) => Ok(Value::bool(*a as f64 >= *b)),
            (Value::f64(a), Value::i64(b)) => Ok(Value::bool(*a >= *b as f64)),
            (Value::i64(a), Value::i64(b)) => Ok(Value::bool(a >= b)),
            _ => Err(ValueErrors::CannotCompareTypes(
                other.to_string(),
                self.to_string(),
            )),
        }
    }

    pub fn try_lt(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::bool(a < b)),
            (Value::i64(a), Value::f64(b)) => Ok(Value::bool((*a as f64) < *b)),
            (Value::f64(a), Value::i64(b)) => Ok(Value::bool(*a < *b as f64)),
            (Value::i64(a), Value::i64(b)) => Ok(Value::bool(a < b)),
            _ => Err(ValueErrors::CannotCompareTypes(
                other.to_string(),
                self.to_string(),
            )),
        }
    }

    pub fn try_lte(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::bool(a <= b)),
            (Value::i64(a), Value::f64(b)) => Ok(Value::bool(*a as f64 <= *b)),
            (Value::f64(a), Value::i64(b)) => Ok(Value::bool(*a <= *b as f64)),
            (Value::i64(a), Value::i64(b)) => Ok(Value::bool(a <= b)),
            _ => Err(ValueErrors::CannotCompareTypes(
                other.to_string(),
                self.to_string(),
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
            (Value::f64(a), Value::Vector(v)) | (Value::Vector(v), Value::f64(a)) => {
                Ok(Value::Vector(Box::new(v.scale(*a))))
            }
            (Value::Matrix(m), Value::f64(f)) | (Value::f64(f), Value::Matrix(m)) => {
                Ok(Value::Matrix(Box::new(m.scale(*f))))
            }
            (Value::i64(a), Value::Vector(v)) | (Value::Vector(v), Value::i64(a)) => {
                Ok(Value::Vector(Box::new(v.scale(*a as f64))))
            }
            (Value::Matrix(m), Value::i64(f)) | (Value::i64(f), Value::Matrix(m)) => {
                Ok(Value::Matrix(Box::new(m.scale(*f as f64))))
            }
            (Value::Matrix(a), Value::Vector(b)) => {
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
                    Ok(Value::Vector(Box::new(result)))
                } else {
                    Err(ValueErrors::SizeMismatch(
                        a.ncols().to_string(),
                        b.len().to_string(),
                    ))
                }
            }
            (Value::Matrix(a), Value::Matrix(b)) => {
                Ok(Value::Matrix(Box::new(*a.clone() * *b.clone())))
            }
            (Value::Quaternion(q1), Value::Quaternion(q2)) => {
                Ok(Value::Quaternion(Box::new(**q1 * **q2)))
            }
            (Value::UnitQuaternion(q1), Value::UnitQuaternion(q2)) => {
                Ok(Value::UnitQuaternion(Box::new(**q1 * **q2)))
            }
            (Value::UnitQuaternion(q1), Value::Quaternion(q2)) => {
                Ok(Value::Quaternion(Box::new(q1.0 * **q2)))
            }
            (Value::Quaternion(q1), Value::UnitQuaternion(q2)) => {
                Ok(Value::Quaternion(Box::new(**q1 * q2.0)))
            }
            _ => Err(ValueErrors::CannotMultiplyTypes(
                other.to_string(),
                self.to_string(),
            )),
        }
    }

    pub fn try_negative(&self) -> Result<Value, ValueErrors> {
        match self {
            Value::f64(v) => Ok(Value::f64(-v)),
            Value::i64(v) => Ok(Value::i64(-v)),
            Value::Vector(v) => Ok(Value::Vector(Box::new(-*v.clone()))),
            Value::Matrix(v) => Ok(Value::Matrix(Box::new(-*v.clone()))),
            Value::Quaternion(v) => Ok(Value::Quaternion(Box::new(-(**v)))),
            Value::UnitQuaternion(v) => Ok(Value::UnitQuaternion(Box::new(-(**v)))),
            _ => Err(ValueErrors::CannotNegType(self.to_string())),
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
            _ => Err(ValueErrors::CannotPowType(self.to_string())),
        }
    }

    pub fn try_sub(&self, other: &Value) -> Result<Value, ValueErrors> {
        match (self, other) {
            (Value::f64(a), Value::f64(b)) => Ok(Value::f64(a - b)),
            (Value::i64(a), Value::f64(b)) => Ok(Value::f64(*a as f64 - b)),
            (Value::f64(a), Value::i64(b)) => Ok(Value::f64(a - *b as f64)),
            (Value::i64(a), Value::i64(b)) => Ok(Value::i64(a - b)),
            (Value::Vector(v), Value::f64(a)) => Ok(Value::Vector(Box::new(v.add_scalar(-*a)))),
            (Value::Vector(v), Value::i64(a)) => {
                Ok(Value::Vector(Box::new(v.add_scalar(-(*a as f64)))))
            }
            (Value::Matrix(m), Value::f64(f)) => Ok(Value::Matrix(Box::new(m.add_scalar(-*f)))),
            (Value::Matrix(m), Value::i64(f)) => {
                Ok(Value::Matrix(Box::new(m.add_scalar(-(*f as f64)))))
            }
            _ => Err(ValueErrors::CannotSubtractTypes(
                other.to_string(),
                self.to_string(),
            )),
        }
    }

    pub fn try_matrix_index(
        &self,
        row_index: IndexStyle,
        col_index: IndexStyle,
    ) -> Result<Value, ValueErrors> {
        match self {
            Value::Matrix(m) => {
                let (rows, cols) = (m.nrows(), m.ncols());

                // Convert IndexStyle to actual row and column indices
                let row_indices = match row_index {
                    IndexStyle::All => (0..rows).collect(),
                    IndexStyle::Usize(r) if r < rows => vec![r],
                    IndexStyle::Usize(r) => return Err(ValueErrors::OutOfBoundsIndex(r, rows)),
                    IndexStyle::Range(r) => {
                        let start = r.start.unwrap_or(0);
                        let stop = r.stop.unwrap_or(rows);
                        if stop > rows {
                            return Err(ValueErrors::OutOfBoundsIndex(stop, rows));
                        }
                        (start..stop).step_by(r.step.unwrap_or(1)).collect()
                    }
                    IndexStyle::VecUsize(indices) => {
                        let indices_vec = indices.iter().copied().collect::<Vec<usize>>(); // Convert DVector to Vec

                        if indices_vec.iter().any(|&r| r >= rows) {
                            return Err(ValueErrors::OutOfBoundsIndex(
                                *indices_vec.iter().max().unwrap(),
                                rows,
                            ));
                        }

                        indices_vec // Return the converted Vec<usize>
                    }
                    IndexStyle::VecBool(mask) => {
                        if mask.len() != rows {
                            return Err(ValueErrors::SizeMismatch(
                                mask.len().to_string(),
                                rows.to_string(),
                            ));
                        }
                        (0..rows).filter(|&i| mask[i]).collect()
                    }
                };

                let col_indices = match col_index {
                    IndexStyle::All => (0..cols).collect(),
                    IndexStyle::Usize(c) if c < cols => vec![c],
                    IndexStyle::Usize(c) => return Err(ValueErrors::OutOfBoundsIndex(c, cols)),
                    IndexStyle::Range(r) => {
                        let start = r.start.unwrap_or(0);
                        let stop = r.stop.unwrap_or(cols);
                        if stop > cols {
                            return Err(ValueErrors::OutOfBoundsIndex(stop, cols));
                        }
                        (start..stop).step_by(r.step.unwrap_or(1)).collect()
                    }
                    IndexStyle::VecUsize(indices) => {
                        let indices_vec = indices.iter().copied().collect::<Vec<usize>>(); // Convert DVector to Vec

                        if indices.iter().any(|&c| c >= cols) {
                            return Err(ValueErrors::OutOfBoundsIndex(
                                *indices.iter().max().unwrap(),
                                cols,
                            ));
                        }
                        indices_vec
                    }
                    IndexStyle::VecBool(mask) => {
                        if mask.len() != cols {
                            return Err(ValueErrors::SizeMismatch(
                                mask.len().to_string(),
                                cols.to_string(),
                            ));
                        }
                        (0..cols).filter(|&i| mask[i]).collect()
                    }
                };

                // Single Element Case
                if row_indices.len() == 1 && col_indices.len() == 1 {
                    return Ok(Value::f64(m[(row_indices[0], col_indices[0])]));
                }

                // Single Row or Column Case (Returns a Vector)
                if row_indices.len() == 1 {
                    let row = m.row(row_indices[0]);
                    return Ok(Value::Vector(Box::new(DVector::from_vec(
                        col_indices.iter().map(|&c| row[c]).collect(),
                    ))));
                }
                if col_indices.len() == 1 {
                    let col = m.column(col_indices[0]);
                    return Ok(Value::Vector(Box::new(DVector::from_vec(
                        row_indices.iter().map(|&r| col[r]).collect(),
                    ))));
                }

                // General Case (Returns a Submatrix)
                let mut submatrix_data = Vec::new();
                for &r in &row_indices {
                    for &c in &col_indices {
                        submatrix_data.push(m[(r, c)]);
                    }
                }

                let submatrix =
                    DMatrix::from_vec(row_indices.len(), col_indices.len(), submatrix_data);
                Ok(Value::Matrix(Box::new(submatrix)))
            }
            _ => Err(ValueErrors::CannotIndexType(self.to_string())),
        }
    }

    pub fn try_vector_index(&self, index: IndexStyle) -> Result<Value, ValueErrors> {
        match self {
            Value::Vector(v) => match index {
                IndexStyle::All => Ok(Value::Vector(Box::new(*v.clone()))),
                IndexStyle::Range(r) => {
                    let start = r.start.unwrap_or(0);
                    let stop = r.stop.unwrap_or(v.len());

                    if stop > v.len() {
                        return Err(ValueErrors::OutOfBoundsIndex(stop, v.len()));
                    }
                    // Generate row indices based on step size
                    let v2 = if let Some(step) = r.step {
                        let indices: Vec<usize> = (start..stop).step_by(step).collect();
                        v.select_rows(&indices) // Efficiently selects stepped elements
                    } else {
                        v.rows(start, stop - start).into() // Standard range without stepping
                    };

                    Ok(Value::Vector(Box::new(v2.into())))
                }
                IndexStyle::Usize(u) => {
                    if u > v.len() {
                        return Err(ValueErrors::OutOfBoundsIndex(u, v.len()));
                    }
                    Ok(Value::f64(v[u]))
                }
                IndexStyle::VecBool(vb) => {
                    if vb.len() != v.len() {
                        return Err(ValueErrors::SizeMismatch(
                            vb.len().to_string(),
                            v.len().to_string(),
                        ));
                    }
                    let mut v2 = Vec::new();
                    for i in 0..vb.len() {
                        if vb[i] {
                            v2.push(v[i]);
                        }
                    }
                    Ok(Value::Vector(Box::new(DVector::from(v2))))
                }
                IndexStyle::VecUsize(vu) => {
                    let mut v2 = Vec::new();
                    let vl = v.len();
                    for i in 0..vu.len() {
                        if vu[i] < vl {
                            v2.push(v[vu[i]]);
                        } else {
                            return Err(ValueErrors::OutOfBoundsIndex(vu[i], vl));
                        }
                    }

                    Ok(Value::Vector(Box::new(DVector::from(v2))))
                }
            },
            _ => Err(ValueErrors::CannotIndexType(self.to_string())),
        }
    }
}

#[derive(Clone, Debug)]
pub enum IndexStyle {
    All,
    Range(Range),
    Usize(usize),
    VecBool(Box<DVector<bool>>),
    VecUsize(Box<DVector<usize>>),
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Range {
    pub start: Option<usize>,
    pub stop: Option<usize>,
    pub step: Option<usize>,
}

#[derive(Clone)]
pub struct Enum {
    pub name: String,
    pub variant: String,
}
