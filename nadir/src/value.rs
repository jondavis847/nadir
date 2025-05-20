use aerospace::orbit::KeplerianElements;
use celestial::CelestialBodies;
use chrono::NaiveDateTime;
use iced::window::Id;
use multibody::system::MultibodySystemBuilder;
use nalgebra::{DMatrix, DVector};
use rotations::{
    //euler_angles::{EulerAngles, EulerSequence},
    prelude::QuaternionErrors,
    quaternion::{Quaternion, UnitQuaternion}, //rotation_matrix::RotationMatrix,
};
use unicode_width::UnicodeWidthStr;

use std::{
    collections::HashMap,
    f64::{INFINITY, NAN},
    path::PathBuf,
    sync::{Arc, Mutex},
};
use thiserror::Error;
use time::{Time, TimeFormat, TimeSystem};

use crate::plotting::{axes::Axes, figure::Figure, line::Line};

pub fn label(s: &str) -> String {
    ansi_term::Colour::Fixed(237).paint(s).to_string()
}

#[derive(Debug, Error)]
pub enum ValueErrors {
    #[error("cannot add type {1} to {0}")]
    CannotAddTypes(String, String),
    #[error("cannot compare type {1} by {0}")]
    CannotCompareTypes(String, String),
    #[error("cannot convert type {0} to type {1}")]
    CannotConvert(String, String),
    #[error("{0}")]
    CannotConvertToBool(String),
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
    #[error("{0}")]
    QuaternionErrors(#[from] QuaternionErrors),
    #[error("dimension mismatch: matrix cols {0}, vector rows {1}")]
    SizeMismatch(String, String),
}

#[derive(Clone)]
#[allow(non_camel_case_types)]
pub enum Value {
    f64(f64),
    i64(i64),
    bool(bool),
    Axes(Arc<Mutex<Axes>>),
    CelestialBodies(CelestialBodies),
    DateTime(NaiveDateTime),
    Event(Event),
    KeplerianElements(KeplerianElements),
    Line(Arc<Mutex<Line>>),
    Matrix(Arc<Mutex<DMatrix<f64>>>),
    MultibodySystemBuilder(Arc<Mutex<MultibodySystemBuilder>>),
    Map(Arc<Mutex<Map>>),
    None,
    Figure(Arc<Mutex<Figure>>),
    Quaternion(Arc<Mutex<Quaternion>>),
    Range(Range),
    String(Arc<Mutex<String>>),
    Time(Arc<Mutex<Time>>),
    TimeFormat(TimeFormat),
    TimeSystem(TimeSystem),
    UnitQuaternion(Arc<Mutex<UnitQuaternion>>),
    Vector(Arc<Mutex<DVector<f64>>>),
    VectorBool(Arc<Mutex<DVector<bool>>>),
    VectorUsize(Arc<Mutex<DVector<usize>>>),
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
            Value::Axes(p) => {
                let axes = &*p.lock().unwrap();
                writeln!(f, "{}", label("Axes"))?;
                writeln!(f, "axis: Axis,")?;
                writeln!(f, "figure_id: {:?},", axes.figure_id)?;
                writeln!(f, "bounds: {:?},", axes.bounds)?;
                writeln!(f, "legend: Legend,")?;
                writeln!(f, "location: {:?},", axes.location)?;
                writeln!(f, "padding: {:?},", axes.padding)?;
                writeln!(f, "xlim: {:?},", axes.xlim)?;
                writeln!(f, "ylim: {:?},", axes.ylim)?;
                if axes.lines.is_empty() {
                    writeln!(f, "lines: [],")?;
                } else {
                    writeln!(f, "lines: [ ")?;
                    for (i, _line) in axes.lines.iter().enumerate() {
                        writeln!(f, "     {} Line,", label(&i.to_string()),)?;
                    }
                    writeln!(f, "   ],")?;
                }
                Ok(())
            }
            Value::CelestialBodies(c) => {
                writeln!(f, "{}{:?}", label("CelestialBodies::"), c)
            }
            Value::DateTime(dt) => {
                writeln!(f, "{:?}", dt)
            }
            Value::Event(e) => writeln!(f, "{:?}", e),
            Value::Figure(p) => {
                let figure = p.lock().unwrap();
                writeln!(f, "{}", label("Figure"))?;
                let id = &match figure.get_id() {
                    Some(id) => id.to_string(),
                    None => "None".to_string(),
                };
                writeln!(f, "id: {},", id)?;
                if figure.axes.is_empty() {
                    writeln!(f, "axes: [],")?;
                } else {
                    writeln!(f, "axes: [ ")?;
                    for (i, axes) in figure.axes.iter().enumerate() {
                        let axes = &*axes.lock().unwrap();
                        writeln!(
                            f,
                            "     {} Axes({},{}),",
                            label(&i.to_string()),
                            axes.location.0,
                            axes.location.1
                        )?;
                    }
                    writeln!(f, "   ],")?;
                }
                Ok(())
            }
            Value::KeplerianElements(ke) => writeln!(f, "{:#?}", ke),
            Value::Line(line) => {
                let line = line.lock().unwrap();
                writeln!(f, "{}", label("Line"))?;
                writeln!(f, "color: {:?}", line.color)?;
                writeln!(f, "width: {:?}", line.width)?;
                writeln!(f, "data: Vector<f64,{}>", line.data.len())
            }
            Value::Map(m) => {
                writeln!(f, "Map")?;
                let map = &m.lock().unwrap().0;

                // Find the length of the longest key
                let max_key_length = map.keys().map(|key| key.len()).max().unwrap_or(0);

                // Format each key-value pair with aligned keys
                for (key, value) in map {
                    writeln!(
                        f,
                        "{:width$} {}",
                        key,
                        label(&value.to_string()),
                        width = max_key_length + 2
                    )?;
                }
                Ok(())
            }
            Value::Matrix(m) => {
                let m = m.lock().unwrap();
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
            Value::MultibodySystemBuilder(m) => writeln!(f, "{:?}", m),
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
                // Helper function to format a single vector element
                fn format_vector_element(
                    f: &mut std::fmt::Formatter<'_>,
                    index: usize,
                    value: f64,
                ) -> std::fmt::Result {
                    // Check if the fractional part is effectively 0
                    if value.fract().abs() < std::f64::EPSILON {
                        // Print with one decimal place (appending ".0")
                        writeln!(f, "{} {:.1}", label(&index.to_string()), value)
                    } else {
                        // Print the full decimal representation
                        writeln!(f, "{} {}", label(&index.to_string()), value)
                    }
                }
                let v = v.lock().unwrap();
                writeln!(f, "{}", label(&format!("Vector<f64,{}>", v.len())))?;

                // Determine how many entries to display
                let len = v.len();

                if len <= 20 {
                    // For smaller vectors, display all elements
                    for (i, e) in v.iter().enumerate() {
                        format_vector_element(f, i, *e)?;
                    }
                } else {
                    // For larger vectors, display first 10, ellipsis, last 10

                    // Display first 10 elements
                    for i in 0..10 {
                        format_vector_element(f, i, v[i])?;
                    }

                    // Display ellipsis
                    writeln!(f, ":")?;

                    // Display last 10 elements
                    for i in (len - 10)..len {
                        format_vector_element(f, i, v[i])?;
                    }
                }

                Ok(())
            }
            Value::VectorUsize(v) => {
                let v = v.lock().unwrap();
                writeln!(f, "{}", label(&format!("Vector<usize,{}>", v.len())))?;
                for (i, e) in v.iter().enumerate() {
                    writeln!(f, "{} {}", label(&i.to_string()), e)?;
                }
                Ok(())
            }
            Value::VectorBool(v) => {
                let v = v.lock().unwrap();
                writeln!(f, "{}", label(&format!("Vector<bool,{}>", v.len())))?;
                for (i, e) in v.iter().enumerate() {
                    writeln!(f, "{} {}", label(&i.to_string()), e)?;
                }
                Ok(())
            }

            Value::String(s) => {
                let s = s.lock().unwrap();
                writeln!(f, "{}", label("String"))?;
                writeln!(f, "{}", s)
            }
            Value::Quaternion(q) => {
                let q = q.lock().unwrap();
                writeln!(f, "{}", label("Quaternion"))?;
                writeln!(f, "{} {}", label("x"), q.x)?;
                writeln!(f, "{} {}", label("y"), q.y)?;
                writeln!(f, "{} {}", label("z"), q.z)?;
                writeln!(f, "{} {}", label("w"), q.w)
            }
            Value::Time(t) => {
                let t = t.lock().unwrap();
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
            Value::TimeFormat(t) => {
                writeln!(f, "{}{:?}", label("TimeFormat::"), t)
            }
            Value::TimeSystem(t) => {
                writeln!(f, "{}{:?}", label("TimeSystem::"), t)
            }
            Value::UnitQuaternion(q) => {
                let q = q.lock().unwrap();
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
            Value::f64(_) => "f64".into(),
            Value::i64(_) => "i64".into(),
            Value::bool(_) => "bool".into(),
            Value::Axes(_) => "Axes".into(),
            Value::DateTime(_) => "DateTime".into(),
            Value::Event(_) => "Event".into(),
            Value::CelestialBodies(_) => "CelestialBodies".into(),
            Value::KeplerianElements(_) => "KeplerianElements".into(),
            Value::Line(_) => "Line".into(),
            Value::Map(_) => "Map".into(),
            Value::MultibodySystemBuilder(_) => "MultibodySystemBuilder".into(),
            Value::Figure(_) => "Figure".into(),
            Value::Vector(v) => {
                let v = v.lock().unwrap();
                let length = v.len();
                String::from(format!("Vector<f64,{}>", length))
            }
            Value::VectorUsize(v) => {
                let v = v.lock().unwrap();
                let length = v.len();
                String::from(format!("Vector<usize,{}>", length))
            }
            Value::VectorBool(v) => {
                let v = v.lock().unwrap();
                let length = v.len();
                String::from(format!("Vector<bool,{}>", length))
            }
            Value::Matrix(v) => {
                let v = v.lock().unwrap();
                let rows = v.nrows();
                let cols = v.ncols();
                String::from(format!("Matrix<f64,{},{}>", rows, cols))
            }
            Value::Time(_) => "Time".into(),
            Value::TimeFormat(_) => "TimeFormat".into(),
            Value::TimeSystem(_) => "TimeSystem".into(),
            Value::None => "None".into(),
            Value::Range(_) => "Range".into(),
            Value::Quaternion(_) => "Quaternion".into(),
            Value::UnitQuaternion(_) => "UnitQuaternion".into(),
            Value::String(_) => "String".into(),
        }
    }

    // returns the general type of the Value
    // we need this since we custom format to_string() to pring the size and type
    pub fn type_check(&self) -> String {
        match self {
            Value::Vector(_) => "Vector".to_string(),
            Value::VectorUsize(_) => "VectorUsize".to_string(),
            Value::VectorBool(_) => "VectorBool".to_string(),
            Value::Matrix(_) => "Matrix".to_string(),
            _ => self.to_string(),
        }
    }

    pub fn as_axes(&self) -> Result<Arc<Mutex<Axes>>, ValueErrors> {
        match self {
            Value::Axes(a) => Ok(Arc::clone(a)),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "Axes".to_string(),
            )),
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

    pub fn as_celestial_body(&self) -> Result<CelestialBodies, ValueErrors> {
        match self {
            Value::CelestialBodies(v) => Ok(*v),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "CelestialBodies".to_string(),
            )),
        }
    }

    pub fn as_f64(&self) -> Result<f64, ValueErrors> {
        match self {
            Value::f64(v) => Ok(*v),
            Value::i64(v) => Ok(*v as f64),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "f64".to_string(),
            )),
        }
    }

    pub fn as_i64(&self) -> Result<i64, ValueErrors> {
        match self {
            Value::f64(v) => Ok(*v as i64),
            Value::i64(v) => Ok(*v),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "i64".to_string(),
            )),
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
            Value::VectorBool(v) => Ok(IndexStyle::VecBool(v.lock().unwrap().clone())),
            Value::VectorUsize(v) => Ok(IndexStyle::VecUsize(v.lock().unwrap().clone())),
            _ => Err(ValueErrors::NonIndexType(self.to_string())),
        }
    }

    pub fn as_keplerian_elements(&self) -> Result<KeplerianElements, ValueErrors> {
        match self {
            Value::KeplerianElements(v) => Ok(v.clone()),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "KeplerianElements".to_string(),
            )),
        }
    }

    pub fn as_line(&self) -> Result<Arc<Mutex<Line>>, ValueErrors> {
        match self {
            Value::Line(l) => Ok(Arc::clone(l)),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "Line".to_string(),
            )),
        }
    }

    pub fn as_figure(&self) -> Result<Arc<Mutex<Figure>>, ValueErrors> {
        match self {
            Value::Figure(p) => Ok(Arc::clone(p)),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "Figure".to_string(),
            )),
        }
    }

    pub fn as_map(&self) -> Result<Arc<Mutex<Map>>, ValueErrors> {
        match self {
            Value::Map(m) => Ok(Arc::clone(m)),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "Map".to_string(),
            )),
        }
    }

    pub fn as_string(&self) -> Result<String, ValueErrors> {
        match self {
            Value::f64(v) => Ok(v.to_string()),
            Value::i64(v) => Ok(v.to_string()),
            Value::String(v) => Ok(v.lock().unwrap().clone()),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "String".to_string(),
            )),
        }
    }

    pub fn as_time(&self) -> Result<Arc<Mutex<Time>>, ValueErrors> {
        match self {
            Value::Time(v) => Ok(v.clone()),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "Time".to_string(),
            )),
        }
    }

    pub fn as_time_format(&self) -> Result<TimeFormat, ValueErrors> {
        match self {
            Value::TimeFormat(v) => Ok(*v),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "TimeFormat".to_string(),
            )),
        }
    }

    pub fn as_time_system(&self) -> Result<TimeSystem, ValueErrors> {
        match self {
            Value::TimeSystem(v) => Ok(*v),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "TimeSystem".to_string(),
            )),
        }
    }

    pub fn as_usize(&self) -> Result<usize, ValueErrors> {
        match self {
            Value::f64(v) => Ok(*v as usize),
            Value::i64(v) => Ok(*v as usize),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "usize".to_string(),
            )),
        }
    }

    pub fn as_quaternion(&self) -> Result<Quaternion, ValueErrors> {
        match self {
            Value::Quaternion(v) => Ok(*v.lock().unwrap()),
            Value::UnitQuaternion(v) => Ok(Quaternion::from(&*v.lock().unwrap())),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "Quaternion".to_string(),
            )),
        }
    }

    pub fn as_unit_quaternion(&self) -> Result<UnitQuaternion, ValueErrors> {
        match self {
            Value::Quaternion(v) => Ok(UnitQuaternion::try_from(&*v.lock().unwrap())?),
            Value::UnitQuaternion(v) => Ok(*v.lock().unwrap()),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "UnitQuaternion".to_string(),
            )),
        }
    }

    pub fn as_vector(&self) -> Result<Arc<Mutex<DVector<f64>>>, ValueErrors> {
        match self {
            Value::Vector(v) => Ok(v.clone()),
            _ => Err(ValueErrors::CannotConvert(
                self.to_string(),
                "usize".to_string(),
            )),
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
                let v = v.lock().unwrap();
                Ok(Value::Vector(Arc::new(Mutex::new(v.add_scalar(*a)))))
            }
            (Value::Matrix(m), Value::f64(f)) | (Value::f64(f), Value::Matrix(m)) => {
                let m = m.lock().unwrap();
                Ok(Value::Matrix(Arc::new(Mutex::new(m.add_scalar(*f)))))
            }
            (Value::i64(a), Value::Vector(v)) | (Value::Vector(v), Value::i64(a)) => {
                let v = v.lock().unwrap();
                Ok(Value::Vector(Arc::new(Mutex::new(v.add_scalar(*a as f64)))))
            }
            (Value::Matrix(m), Value::i64(f)) | (Value::i64(f), Value::Matrix(m)) => {
                let m = m.lock().unwrap();
                Ok(Value::Matrix(Arc::new(Mutex::new(m.add_scalar(*f as f64)))))
            }
            (Value::Vector(v1), Value::Vector(v2)) => {
                let v1 = v1.lock().unwrap();
                let v2 = v2.lock().unwrap();
                if v1.len() != v2.len() {
                    return Err(ValueErrors::SizeMismatch(
                        v1.len().to_string(),
                        v2.len().to_string(),
                    ));
                }
                let mut v3 = v1.clone();
                for i in 0..v1.len() {
                    v3[i] = v1[i] + v2[i];
                }
                Ok(Value::Vector(Arc::new(Mutex::new(v3))))
            }
            (Value::Matrix(v1), Value::Matrix(v2)) => {
                let v1 = v1.lock().unwrap();
                let v2 = v2.lock().unwrap();
                if v1.nrows() != v2.nrows() || v1.ncols() != v2.ncols() {
                    return Err(ValueErrors::SizeMismatch(
                        format!("{}x{}", v1.nrows(), v1.ncols()),
                        format!("{}x{}", v2.nrows(), v2.ncols()),
                    ));
                }
                let mut v3 = v1.clone();
                for i in 0..v1.nrows() {
                    for j in 0..v1.ncols() {
                        v3[(i, j)] = v1[(i, j)] + v2[(i, j)];
                    }
                }
                Ok(Value::Matrix(Arc::new(Mutex::new(v3))))
            }
            (Value::String(a), Value::String(b)) => {
                let a = a.lock().unwrap();
                let b = b.lock().unwrap();
                Ok(Value::String(Arc::new(Mutex::new(format!("{}{}", a, b)))))
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
                let v = v.lock().unwrap();
                Ok(Value::Vector(Arc::new(Mutex::new(v.scale(1.0 / *a))))) // This will produce inf or NaN elements for division by zero
            }
            (Value::Matrix(m), Value::f64(f)) => {
                let m = m.lock().unwrap();
                Ok(Value::Matrix(Arc::new(Mutex::new(m.scale(1.0 / *f))))) // This will produce inf or NaN elements for division by zero
            }
            (Value::Vector(v), Value::i64(a)) => {
                let v = v.lock().unwrap();
                if *a == 0 {
                    Ok(Value::Vector(Arc::new(Mutex::new(v.map(|x| {
                        if x == 0.0 {
                            NAN
                        } else if x > 0.0 {
                            INFINITY
                        } else {
                            -INFINITY
                        }
                    })))))
                } else {
                    Ok(Value::Vector(Arc::new(Mutex::new(
                        v.scale(1.0 / *a as f64),
                    ))))
                }
            }
            (Value::Matrix(m), Value::i64(f)) => {
                let m = m.lock().unwrap();
                if *f == 0 {
                    Ok(Value::Matrix(Arc::new(Mutex::new(m.map(|x| {
                        if x == 0.0 {
                            NAN
                        } else if x > 0.0 {
                            INFINITY
                        } else {
                            -INFINITY
                        }
                    })))))
                } else {
                    Ok(Value::Matrix(Arc::new(Mutex::new(
                        m.scale(1.0 / *f as f64),
                    ))))
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
                let v = v.lock().unwrap();
                Ok(Value::Vector(Arc::new(Mutex::new(v.scale(*a)))))
            }
            (Value::Matrix(m), Value::f64(f)) | (Value::f64(f), Value::Matrix(m)) => {
                let m = m.lock().unwrap();
                Ok(Value::Matrix(Arc::new(Mutex::new(m.scale(*f)))))
            }
            (Value::i64(a), Value::Vector(v)) | (Value::Vector(v), Value::i64(a)) => {
                let v = v.lock().unwrap();
                Ok(Value::Vector(Arc::new(Mutex::new(v.scale(*a as f64)))))
            }
            (Value::Matrix(m), Value::i64(f)) | (Value::i64(f), Value::Matrix(m)) => {
                let m = m.lock().unwrap();
                Ok(Value::Matrix(Arc::new(Mutex::new(m.scale(*f as f64)))))
            }
            (Value::Matrix(a), Value::Vector(b)) => {
                let a = &*a.lock().unwrap();
                let b = &*b.lock().unwrap();

                // Check that the matrix and vector dimensions are compatible.
                // (For a multiplication a * b to work, the number of columns of 'a' must equal the number of rows of 'b',
                // which is equivalent to b.len() for a column vector.)
                if a.ncols() == b.len() {
                    let result: DVector<f64> = a * b;
                    Ok(Value::Vector(Arc::new(Mutex::new(result))))
                } else {
                    Err(ValueErrors::SizeMismatch(
                        a.ncols().to_string(),
                        b.len().to_string(),
                    ))
                }
            }
            (Value::Matrix(a), Value::Matrix(b)) => {
                let a = &*a.lock().unwrap();
                let b = &*b.lock().unwrap();
                Ok(Value::Matrix(Arc::new(Mutex::new(a * b))))
            }
            (Value::Quaternion(q1), Value::Quaternion(q2)) => {
                let q1 = *q1.lock().unwrap();
                let q2 = *q2.lock().unwrap();
                Ok(Value::Quaternion(Arc::new(Mutex::new(q1 * q2))))
            }
            (Value::UnitQuaternion(q1), Value::UnitQuaternion(q2)) => {
                let q1 = *q1.lock().unwrap();
                let q2 = *q2.lock().unwrap();
                Ok(Value::UnitQuaternion(Arc::new(Mutex::new(q1 * q2))))
            }
            (Value::UnitQuaternion(q1), Value::Quaternion(q2)) => {
                let q1 = *q1.lock().unwrap();
                let q2 = *q2.lock().unwrap();
                Ok(Value::Quaternion(Arc::new(Mutex::new(q1.0 * q2))))
            }
            (Value::Quaternion(q1), Value::UnitQuaternion(q2)) => {
                let q1 = *q1.lock().unwrap();
                let q2 = *q2.lock().unwrap();
                Ok(Value::Quaternion(Arc::new(Mutex::new(q1 * q2.0))))
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
            Value::Vector(v) => {
                let v = &*v.lock().unwrap();
                Ok(Value::Vector(Arc::new(Mutex::new(-v))))
            }
            Value::Matrix(v) => {
                let v = &*v.lock().unwrap();
                Ok(Value::Matrix(Arc::new(Mutex::new(-v))))
            }
            Value::Quaternion(v) => {
                let v = *v.lock().unwrap();
                Ok(Value::Quaternion(Arc::new(Mutex::new(-v))))
            }
            Value::UnitQuaternion(v) => {
                let v = *v.lock().unwrap();
                Ok(Value::UnitQuaternion(Arc::new(Mutex::new(-v))))
            }
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
            (Value::Vector(v), Value::f64(a)) => {
                let v = &*v.lock().unwrap();
                Ok(Value::Vector(Arc::new(Mutex::new(v.add_scalar(-*a)))))
            }
            (Value::Vector(v), Value::i64(a)) => {
                let v = &*v.lock().unwrap();
                Ok(Value::Vector(Arc::new(Mutex::new(
                    v.add_scalar(-(*a as f64)),
                ))))
            }
            (Value::Matrix(m), Value::f64(f)) => {
                let m = &*m.lock().unwrap();
                Ok(Value::Matrix(Arc::new(Mutex::new(m.add_scalar(-*f)))))
            }
            (Value::Matrix(m), Value::i64(f)) => {
                let m = &*m.lock().unwrap();
                Ok(Value::Matrix(Arc::new(Mutex::new(
                    m.add_scalar(-(*f as f64)),
                ))))
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
                let m = &*m.lock().unwrap();
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
                    return Ok(Value::Vector(Arc::new(Mutex::new(DVector::from_vec(
                        col_indices.iter().map(|&c| row[c]).collect(),
                    )))));
                }
                if col_indices.len() == 1 {
                    let col = m.column(col_indices[0]);
                    return Ok(Value::Vector(Arc::new(Mutex::new(DVector::from_vec(
                        row_indices.iter().map(|&r| col[r]).collect(),
                    )))));
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
                Ok(Value::Matrix(Arc::new(Mutex::new(submatrix))))
            }
            _ => Err(ValueErrors::CannotIndexType(self.to_string())),
        }
    }

    pub fn try_vector_index(&self, index: IndexStyle) -> Result<Value, ValueErrors> {
        match self {
            Value::Vector(v) => {
                let v = &*v.lock().unwrap();
                match index {
                    IndexStyle::All => Ok(Value::Vector(Arc::new(Mutex::new(v.clone())))),
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

                        Ok(Value::Vector(Arc::new(Mutex::new(v2))))
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
                        Ok(Value::Vector(Arc::new(Mutex::new(DVector::from(v2)))))
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

                        Ok(Value::Vector(Arc::new(Mutex::new(DVector::from(v2)))))
                    }
                }
            }
            _ => Err(ValueErrors::CannotIndexType(self.to_string())),
        }
    }
}

#[derive(Clone, Debug)]
pub enum IndexStyle {
    All,
    Range(Range),
    Usize(usize),
    VecBool(DVector<bool>),
    VecUsize(DVector<usize>),
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Range {
    pub start: Option<usize>,
    pub stop: Option<usize>,
    pub step: Option<usize>,
}

#[derive(Debug, Clone)]
pub enum Event {
    Animate(Arc<Mutex<PathBuf>>),
    ClearCache(Id),
    CloseAllFigures,
    NewFigure(Arc<Mutex<Figure>>),
}

#[derive(Debug, Clone, Error)]
pub enum LinspaceErrors {
    #[error("linspace step must be less than stop - start ")]
    StepTooBig,
    #[error("linspace step must be greater than 0")]
    StepTooSmall,
    #[error("linspace step is in wrong direction")]
    StepWrongDirection,
}

#[derive(Debug, Clone, Copy)]
pub struct Linspace {
    start: f64,
    stop: f64,
    step: f64,
}

impl Linspace {
    pub fn new(start: f64, stop: f64, step: f64) -> Result<Self, LinspaceErrors> {
        if step.abs() < std::f64::EPSILON {
            return Err(LinspaceErrors::StepTooSmall);
        }
        if step.abs() > (stop - start).abs() {
            return Err(LinspaceErrors::StepTooBig);
        }
        if (step > 0.0 && start > stop) | (step < 0.0 && start < stop) {
            return Err(LinspaceErrors::StepWrongDirection);
        }
        Ok(Self { start, stop, step })
    }

    pub fn to_vec(&self) -> DVector<f64> {
        let n = ((self.stop - self.start) / self.step).ceil() as usize;
        let mut v = Vec::with_capacity(n);
        v.push(self.start);
        for _ in 1..=n {
            v.push(v.last().unwrap() + self.step);
        }
        // add an extra point if we didn't quite make it to stop
        if v[n] < self.stop {
            v.push(v[n] + self.step);
        }
        // truncate if we went too far
        if v[n] > self.stop {
            v[n] = self.stop;
        }
        DVector::from(v)
    }
}

#[derive(Clone)]
pub struct Map(pub HashMap<String, Value>);

impl Map {
    pub fn new() -> Self {
        Self(HashMap::new())
    }

    pub fn insert(&mut self, key: String, value: Value) {
        self.0.insert(key, value);
    }
}
