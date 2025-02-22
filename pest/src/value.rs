use nalgebra::{DMatrix, DVector};
use rotations::{
    //euler_angles::{EulerAngles, EulerSequence},
    quaternion::{Quaternion, UnitQuaternion},
    //rotation_matrix::RotationMatrix,
};

use std::f64::{INFINITY, NAN};
use thiserror::Error;

fn label(s: &str) -> String {
    ansi_term::Colour::Fixed(238).paint(s).to_string()
}

#[derive(Debug, Error)]
pub enum ValueErrors {
    #[error("cannot add type {1} to {0}")]
    CannotAddTypes(String, String),
    #[error("cannot convert type {0} to f64")]
    CannotConvertToF64(String),
    #[error("cannot convert type {0} to i64")]
    CannotConvertToI64(String),
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
    Vector(Box<DVector<f64>>),
    VectorBool(Box<DVector<bool>>),
    VectorUsize(Box<DVector<usize>>),
    Matrix(Box<DMatrix<f64>>),
    None,
    Range(Range),
    Quaternion(Box<Quaternion>),
    UnitQuaternion(Box<UnitQuaternion>),
    String(Box<String>),
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
            Value::None => writeln!(f, "{}", label("None")),
            Value::Range(r) => {
                writeln!(f, "{}", label("Range"));
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
                writeln!(f, "start: {}", start);
                writeln!(f, "stop: {}", stop);
                writeln!(f, "step: {}", step)
            }
            Value::Vector(v) => {
                writeln!(f, "{}", label(&format!("Vector<f64,{}>", v.len())))?;
                for e in v.iter() {
                    // Check if the fractional part is effectively 0
                    if e.fract().abs() < std::f64::EPSILON {
                        // Print with one decimal place (appending ".0")
                        writeln!(f, "   {:.1}", e)?;
                    } else {
                        // Print the full decimal representation
                        writeln!(f, "   {}", e)?;
                    }
                }
                Ok(())
            }
            Value::VectorUsize(v) => {
                writeln!(f, "{}", label(&format!("Vector<usize,{}>", v.len())))?;
                for e in v.iter() {
                    writeln!(f, "   {}", e)?;
                }
                Ok(())
            }
            Value::VectorBool(v) => {
                writeln!(f, "{}", label(&format!("Vector<bool,{}>", v.len())))?;
                for e in v.iter() {
                    writeln!(f, "   {}", e)?;
                }
                Ok(())
            }
            Value::Matrix(m) => {
                writeln!(
                    f,
                    "{}",
                    label(&format!("Matrix<f64,{}x{}>", m.nrows(), m.ncols()))
                )?;
                for i in 0..m.nrows() {
                    write!(f, "  ")?; // indent each row
                    for j in 0..m.ncols() {
                        let value = m[(i, j)];
                        // Use a small epsilon to account for floating-point precision
                        if value.fract().abs() < 1e-6 {
                            // Print with one decimal place (i.e. ".0"), keeping a width of 8 characters
                            write!(f, "{:8.1} ", value)?;
                        } else {
                            // Otherwise, print with three decimal places and the same width
                            write!(f, "{:8.3} ", value)?;
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
    pub fn as_str(&self) -> String {
        match self {
            Value::f64(_) => String::from("f64"),
            Value::i64(_) => String::from("i64"),
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
            Value::None => "None".to_string(),
            Value::Range(_) => String::from("Range"),
            Value::Quaternion(_) => String::from("Quaternion"),
            Value::UnitQuaternion(_) => String::from("UnitQuaternion"),
            Value::String(_) => String::from("String"),
        }
    }

    pub fn as_f64(&self) -> Result<f64, ValueErrors> {
        match self {
            Value::f64(v) => Ok(*v),
            Value::i64(v) => Ok(*v as f64),
            _ => Err(ValueErrors::CannotConvertToF64(self.as_str())),
        }
    }

    pub fn as_i64(&self) -> Result<i64, ValueErrors> {
        match self {
            Value::f64(v) => Ok(*v as i64),
            Value::i64(v) => Ok(*v),
            _ => Err(ValueErrors::CannotConvertToI64(self.as_str())),
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
            _ => Err(ValueErrors::NonIndexType(self.as_str().to_string())),
        }
    }

    pub fn as_usize(&self) -> Result<usize, ValueErrors> {
        match self {
            Value::f64(v) => Ok(*v as usize),
            Value::i64(v) => Ok(*v as usize),
            _ => Err(ValueErrors::CannotConvertToUsize(self.as_str())),
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
            _ => Err(ValueErrors::CannotAddTypes(other.as_str(), self.as_str())),
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
                other.as_str(),
                self.as_str(),
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
            _ => Err(ValueErrors::CannotFactorialType(self.as_str())),
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
                other.as_str(),
                self.as_str(),
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
            _ => Err(ValueErrors::CannotNegType(self.as_str())),
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
                other.as_str(),
                self.as_str(),
            )),
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
            _ => Err(ValueErrors::CannotIndexType(self.as_str())),
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
