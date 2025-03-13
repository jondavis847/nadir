use nalgebra::{DMatrix, DVector};
use rand::{thread_rng, Rng};
use rand_distr::{Distribution, Normal};
use rotations::prelude::{Quaternion, UnitQuaternion};
use std::collections::HashMap;
use thiserror::Error;
use time::{Time, TimeErrors};

use crate::value::{Value, ValueErrors};

#[derive(Debug, Error)]
pub enum RegistryErrors {
    #[error("incorrect number of arguments for {0}. got {2}, expected {1}")]
    NumberOfArgs(String, String, String),
    #[error("struct '{0}' not found")]
    StructNotFound(String),
    #[error("method '{1}' not found for struct '{0}'")]
    MethodNotFound(String, String),
    #[error("{0}")]
    TimeErrors(#[from] TimeErrors),
    #[error("{0}")]
    ValueErrors(#[from] ValueErrors),
}

pub struct Registry {
    pub structs: HashMap<&'static str, Struct>,
    pub functions: HashMap<&'static str, Vec<Method>>,
}

#[derive(Default)]
pub struct Struct {
    pub methods: HashMap<&'static str, Vec<Method>>,
}

impl Struct {
    fn new(methods: HashMap<&'static str, Vec<Method>>) -> Self {
        Self { methods }
    }
}

#[derive(Default)]
pub struct Method {
    pub args: Vec<Argument>,
}
impl Method {
    fn new(args: Vec<Argument>) -> Self {
        Self { args }
    }
}

pub struct Argument {
    pub name: &'static str,
    pub type_name: &'static str,
}

impl Argument {
    fn new(name: &'static str, type_name: &'static str) -> Self {
        Self { name, type_name }
    }
}

impl Default for Registry {
    fn default() -> Self {
        // Structs with their methods
        let mut structs = HashMap::new();

        // Matrix
        let mut matrix_methods = HashMap::new();
        matrix_methods.insert(
            "rand",
            vec![
                Method::new(vec![Argument::new("n", "i64")]),
                Method::new(vec![Argument::new("m", "i64"), Argument::new("n", "i64")]),
            ],
        );
        matrix_methods.insert("randn", vec![Method::new(vec![Argument::new("n", "i64")])]);
        structs.insert("Matrix", Struct::new(matrix_methods));

        // Quaternion
        let mut quaternion_methods = HashMap::new();
        quaternion_methods.insert(
            "new",
            vec![Method::new(vec![
                Argument::new("x", "f64"),
                Argument::new("y", "f64"),
                Argument::new("z", "f64"),
                Argument::new("w", "f64"),
            ])],
        );
        quaternion_methods.insert("rand", vec![Method::default()]);
        structs.insert("Quaternion", Struct::new(quaternion_methods));

        // Time
        let mut time_methods = HashMap::new();
        time_methods.insert("new", vec![Method::default()]);
        time_methods.insert("now", vec![Method::default()]);
        structs.insert("Time", Struct::new(time_methods));

        // UnitQuaternion
        // Quaternion
        let mut unit_quaternion_methods = HashMap::new();
        unit_quaternion_methods.insert(
            "new",
            vec![Method::new(vec![
                Argument::new("x", "f64"),
                Argument::new("y", "f64"),
                Argument::new("z", "f64"),
                Argument::new("w", "f64"),
            ])],
        );
        unit_quaternion_methods.insert("rand", vec![Method::default()]);
        structs.insert("UnitQuaternion", Struct::new(unit_quaternion_methods));

        // Vector
        let mut vector_methods = HashMap::new();
        vector_methods.insert("rand", vec![Method::new(vec![Argument::new("n", "i64")])]);
        vector_methods.insert("randn", vec![Method::new(vec![Argument::new("n", "i64")])]);

        structs.insert("Vector", Struct::new(vector_methods));

        // Standalone Functions
        let mut functions = HashMap::new();

        Self { structs, functions }
    }
}

impl Registry {
    pub fn eval_struct_method(
        &self,
        struct_name: &str,
        method_name: &str,
        args: Vec<Value>,
    ) -> Result<Value, RegistryErrors> {
        // Get the struct from the registry, or return an error if not found
        let struc = self
            .structs
            .get(struct_name)
            .ok_or_else(|| RegistryErrors::StructNotFound(struct_name.to_string()))?;

        // Get the list of possible method signatures
        let method_overloads = struc.methods.get(method_name).ok_or_else(|| {
            RegistryErrors::MethodNotFound(struct_name.to_string(), method_name.to_string())
        })?;

        // Try each possible overload
        for method in method_overloads {
            // 1) Check if argument lengths match
            if method.args.len() != args.len() {
                continue; // mismatch, try next overload
            }

            // 2) Check if each argument type matches
            let mut signature_matches = true;
            for (required, actual) in method.args.iter().zip(args.iter()) {
                // For example, if `required.type_name` is "f64", check the actual Value
                // In your original code, you used `required.type_name.to_string() == r.to_string()`
                // (Adjust to your actual logic of type matching.)
                if required.type_name != actual.to_string() {
                    signature_matches = false;
                    break;
                }
            }

            if !signature_matches {
                continue; // mismatch, try next overload
            }

            // 3) If we get here, we've found a matching signature. Evaluate the method.
            return match (struct_name, method_name) {
                // -- Matrix methods --
                ("Matrix", "rand") => {
                    let (rows, cols) = match args.len() {
                        1 => (args[0].as_usize()?, args[0].as_usize()?),
                        2 => (args[0].as_usize()?, args[1].as_usize()?),
                        _ => unreachable!("arg size must have matched by now"),
                    };

                    let mut rng = rand::thread_rng();
                    let data: Vec<f64> = (0..(rows * cols)).map(|_| rng.gen()).collect();
                    let matrix = DMatrix::from_vec(rows, cols, data);
                    Ok(Value::Matrix(Box::new(matrix)))
                }
                ("Matrix", "randn") => {
                    let rows = args[0].as_usize()?;
                    let cols = args[1].as_usize()?;

                    let mut rng = thread_rng();
                    let normal = Normal::new(0.0, 1.0).unwrap();

                    let data: Vec<f64> = (0..(rows * cols))
                        .map(|_| normal.sample(&mut rng))
                        .collect();

                    let matrix = DMatrix::from_vec(rows, cols, data);
                    Ok(Value::Matrix(Box::new(matrix)))
                }

                // -- Quaternion methods --
                ("Quaternion", "new") => {
                    let mut quat_args = [0.0; 4];
                    for (i, arg) in args.iter().enumerate() {
                        quat_args[i] = arg.as_f64()?;
                    }
                    Ok(Value::Quaternion(Box::new(Quaternion::new(
                        quat_args[0],
                        quat_args[1],
                        quat_args[2],
                        quat_args[3],
                    ))))
                }
                ("Quaternion", "rand") => Ok(Value::Quaternion(Box::new(Quaternion::rand()))),

                // -- Time --
                ("Time", "now") => Ok(Value::Time(Box::new(Time::now()?))),

                // -- UnitQuaternion methods --
                ("UnitQuaternion", "new") => {
                    let mut quat_args = [0.0; 4];
                    for (i, arg) in args.iter().enumerate() {
                        quat_args[i] = arg.as_f64()?;
                    }
                    Ok(Value::UnitQuaternion(Box::new(UnitQuaternion::new(
                        quat_args[0],
                        quat_args[1],
                        quat_args[2],
                        quat_args[3],
                    ))))
                }
                ("UnitQuaternion", "rand") => {
                    Ok(Value::UnitQuaternion(Box::new(UnitQuaternion::rand())))
                }

                // -- Vector methods --
                ("Vector", "rand") => {
                    let mut rng = rand::thread_rng();
                    let vector =
                        DVector::from_vec((0..args[0].as_usize()?).map(|_| rng.gen()).collect());
                    Ok(Value::Vector(Box::new(vector)))
                }
                ("Vector", "randn") => {
                    let size = args[0].as_usize()?;

                    let mut rng = thread_rng();
                    let normal = Normal::new(0.0, 1.0).unwrap();
                    let vector =
                        DVector::from_vec((0..size).map(|_| normal.sample(&mut rng)).collect());
                    Ok(Value::Vector(Box::new(vector)))
                }

                // Should never happen if struct_name and method_name exist
                _ => unreachable!(
                    "we checked if struct and method exist; if you got here there's a bug"
                ),
            };
        }

        // If we exhaust all overloads without finding a match:
        Err(RegistryErrors::MethodNotFound(
            struct_name.to_string(),
            method_name.to_string(),
        ))
    }

    pub fn get_structs(&self) -> Vec<&'static str> {
        self.structs.keys().into_iter().cloned().collect()
    }

    pub fn get_struct_methods(&self, struc: &str) -> Result<Vec<&'static str>, RegistryErrors> {
        if let Some(struc) = self.structs.get(struc) {
            let methods: Vec<&'static str> = struc.methods.keys().into_iter().cloned().collect();
            Ok(methods)
        } else {
            Err(RegistryErrors::StructNotFound(struc.to_string()))
        }
    }
}
