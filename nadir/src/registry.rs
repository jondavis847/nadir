use nalgebra::{DMatrix, DVector};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use rotations::prelude::{Quaternion, QuaternionErrors, UnitQuaternion};
use std::{
    collections::HashMap,
    sync::{Arc, Mutex},
};
use thiserror::Error;
use time::TimeErrors;

use crate::value::{Event, Linspace, LinspaceErrors, Map, Value, ValueErrors};

#[derive(Debug, Error)]
pub enum RegistryErrors {
    #[error("function '{0}' not found")]
    FunctionNotFound(String),
    #[error("struct '{0}' not found")]
    StructNotFound(String),
    #[error("{0}")]
    LinspaceErrors(#[from] LinspaceErrors),
    #[error("method '{1}' not found for struct '{0}'")]
    MethodNotFound(String, String),
    #[error("{0}")]
    QuaternionErrors(#[from] QuaternionErrors),
    #[error("{0}")]
    TimeErrors(#[from] TimeErrors),
    #[error("{0}")]
    ValueErrors(#[from] ValueErrors),
}

// Types for method implementations
type FunctionFn = fn(Vec<Value>) -> Result<Value, RegistryErrors>;
type StructMethodFn = fn(Vec<Value>) -> Result<Value, RegistryErrors>;
type InstanceMethodFn = fn(Value, Vec<Value>) -> Result<Value, RegistryErrors>;

#[derive(Default, Debug)]
pub struct Struct {
    pub struct_methods: HashMap<&'static str, Vec<StructMethod>>,
    pub instance_methods: HashMap<&'static str, Vec<InstanceMethod>>,
}

impl Struct {
    fn new(
        struct_methods: HashMap<&'static str, Vec<StructMethod>>,
        instance_methods: HashMap<&'static str, Vec<InstanceMethod>>,
    ) -> Self {
        Self {
            struct_methods,
            instance_methods,
        }
    }
}

#[derive(Debug)]
pub struct StructMethod {
    pub args: Vec<Argument>,
    pub implementation: StructMethodFn,
}
impl StructMethod {
    fn new(args: Vec<Argument>, implementation: StructMethodFn) -> Self {
        Self {
            args,
            implementation,
        }
    }
}

#[derive(Debug)]
pub struct InstanceMethod {
    pub args: Vec<Argument>,
    pub implementation: InstanceMethodFn,
}

impl InstanceMethod {
    fn new(args: Vec<Argument>, implementation: InstanceMethodFn) -> Self {
        Self {
            args,
            implementation,
        }
    }
}

#[derive(Debug)]
pub struct FunctionMethod {
    pub args: Vec<Argument>,
    pub implementation: FunctionFn,
}
impl FunctionMethod {
    fn new(args: Vec<Argument>, implementation: FunctionFn) -> Self {
        Self {
            args,
            implementation,
        }
    }
}

#[derive(Debug)]
pub struct Argument {
    pub name: &'static str,
    pub type_name: &'static str,
}

impl Argument {
    fn new(name: &'static str, type_name: &'static str) -> Self {
        Self { name, type_name }
    }
}

pub struct Registry {
    pub structs: HashMap<&'static str, Struct>,
    pub functions: HashMap<&'static str, Vec<FunctionMethod>>,
}

impl Registry {
    pub fn new() -> Self {
        // Structs with their methods
        let mut structs = HashMap::new();

        // Linspace
        let mut linspace_struct_methods = HashMap::new();
        linspace_struct_methods.insert(
            "new",
            vec![StructMethod::new(
                vec![
                    Argument::new("start", "f64"),
                    Argument::new("stop", "f64"),
                    Argument::new("step", "f64"),
                ],
                |args| {
                    let start = args[0].as_f64()?;
                    let stop = args[1].as_f64()?;
                    let step = args[2].as_f64()?;
                    let v = Linspace::new(start, stop, step)?.to_vec();
                    Ok(Value::Vector(Arc::new(Mutex::new(v))))
                },
            )],
        );

        let linspace_instance_methods = HashMap::new();
        structs.insert(
            "Linspace",
            Struct::new(linspace_struct_methods, linspace_instance_methods),
        );

        // Map
        let mut map_struct_methods = HashMap::new();
        map_struct_methods.insert(
            "new",
            vec![StructMethod::new(vec![], |_args| {
                Ok(Value::Map(Arc::new(Mutex::new(Map::new()))))
            })],
        );
        let mut map_instance_methods = HashMap::new();
        map_instance_methods.insert(
            "insert",
            vec![InstanceMethod::new(
                vec![
                    Argument::new("key", "String"),
                    Argument::new("value", "Value"),
                ],
                |val, args| {
                    let map_guard = val.as_map()?;
                    let mut map = map_guard.lock().unwrap();
                    let value = Value::from(args[1].clone());
                    map.0.insert(args[0].as_string()?, value);
                    Ok(Value::None)
                },
            )],
        );
        map_instance_methods.insert(
            "keys",
            vec![InstanceMethod::new(vec![], |val, _args| {
                let map_guard = val.as_map()?;
                let map = map_guard.lock().unwrap();
                for key in map.0.keys() {
                    println!("{key}");
                }
                Ok(Value::None)
            })],
        );
        structs.insert("Map", Struct::new(map_struct_methods, map_instance_methods));

        // Matrix
        let mut matrix_struct_methods = HashMap::new();
        matrix_struct_methods.insert(
            "rand",
            vec![
                StructMethod::new(vec![Argument::new("n", "i64")], |args| {
                    let n = args[0].as_usize()?;
                    let mut rng = rand::rng();
                    let data: Vec<f64> = (0..(n * n)).map(|_| rng.random()).collect();
                    let matrix = DMatrix::from_vec(n, n, data);
                    Ok(Value::Matrix(Arc::new(Mutex::new(matrix))))
                }),
                StructMethod::new(
                    vec![Argument::new("m", "i64"), Argument::new("n", "i64")],
                    |args| {
                        let rows = args[0].as_usize()?;
                        let cols = args[1].as_usize()?;
                        let mut rng = rand::rng();
                        let data: Vec<f64> = (0..(rows * cols)).map(|_| rng.random()).collect();
                        let matrix = DMatrix::from_vec(rows, cols, data);
                        Ok(Value::Matrix(Arc::new(Mutex::new(matrix))))
                    },
                ),
            ],
        );
        matrix_struct_methods.insert(
            "randn",
            vec![
                StructMethod::new(vec![Argument::new("n", "i64")], |args| {
                    let n = args[0].as_usize()?;
                    let mut rng = rand::rng();
                    let normal = Normal::new(0.0, 1.0 / 3.0).unwrap();
                    let data: Vec<f64> = (0..(n * n)).map(|_| normal.sample(&mut rng)).collect();
                    let matrix = DMatrix::from_vec(n, n, data);
                    Ok(Value::Matrix(Arc::new(Mutex::new(matrix))))
                }),
                StructMethod::new(
                    vec![Argument::new("m", "i64"), Argument::new("n", "i64")],
                    |args| {
                        let rows = args[0].as_usize()?;
                        let cols = args[1].as_usize()?;
                        let mut rng = rand::rng();
                        let normal = Normal::new(0.0, 1.0 / 3.0).unwrap();
                        let data: Vec<f64> = (0..(rows * cols))
                            .map(|_| normal.sample(&mut rng))
                            .collect();
                        let matrix = DMatrix::from_vec(rows, cols, data);
                        Ok(Value::Matrix(Arc::new(Mutex::new(matrix))))
                    },
                ),
            ],
        );
        let matrix_instance_methods = HashMap::new();
        structs.insert(
            "Matrix",
            Struct::new(matrix_struct_methods, matrix_instance_methods),
        );

        // Quaternion
        let mut quaternion_struct_methods = HashMap::new();
        quaternion_struct_methods.insert(
            "new",
            vec![StructMethod::new(
                vec![
                    Argument::new("x", "f64"),
                    Argument::new("y", "f64"),
                    Argument::new("z", "f64"),
                    Argument::new("w", "f64"),
                ],
                |args| {
                    Ok(Value::Quaternion(Arc::new(Mutex::new(Quaternion::new(
                        args[0].as_f64()?,
                        args[1].as_f64()?,
                        args[2].as_f64()?,
                        args[3].as_f64()?,
                    )))))
                },
            )],
        );
        quaternion_struct_methods.insert(
            "rand",
            vec![StructMethod::new(vec![], |_| {
                Ok(Value::Quaternion(Arc::new(Mutex::new(Quaternion::rand()))))
            })],
        );
        let mut quaternion_instance_methods = HashMap::new();
        quaternion_instance_methods.insert(
            "inv",
            vec![InstanceMethod::new(vec![], |val, _| {
                Ok(Value::Quaternion(Arc::new(Mutex::new(
                    val.as_quaternion()?.inv(),
                ))))
            })],
        );
        // MultibodySystem
        let mut multibody_system_methods = HashMap::new();
        multibody_system_methods.insert(
            "load",
            vec![StructMethod::new(
                vec![Argument::new("file", "String")],
                |args| {
                    let file = std::env::current_dir().unwrap().join(args[0].as_string()?);

                    Ok(Value::None)
                },
            )],
        );

        structs.insert(
            "Quaternion",
            Struct::new(quaternion_struct_methods, quaternion_instance_methods),
        );

        // Time
        let mut time_struct_methods = HashMap::new();
        time_struct_methods.insert("new", vec![]);
        time_struct_methods.insert("now", vec![]);

        let time_instance_methods = HashMap::new();
        structs.insert(
            "Time",
            Struct::new(time_struct_methods, time_instance_methods),
        );

        // UnitQuaternion
        // Quaternion
        let mut unit_quaternion_struct_methods = HashMap::new();
        unit_quaternion_struct_methods.insert(
            "new",
            vec![StructMethod::new(
                vec![
                    Argument::new("x", "f64"),
                    Argument::new("y", "f64"),
                    Argument::new("z", "f64"),
                    Argument::new("w", "f64"),
                ],
                |args| {
                    Ok(Value::UnitQuaternion(Arc::new(Mutex::new(
                        UnitQuaternion::new(
                            args[0].as_f64()?,
                            args[1].as_f64()?,
                            args[2].as_f64()?,
                            args[3].as_f64()?,
                        )?,
                    ))))
                },
            )],
        );
        unit_quaternion_struct_methods.insert(
            "rand",
            vec![StructMethod::new(vec![], |_args| {
                Ok(Value::Quaternion(Arc::new(Mutex::new(Quaternion::rand()))))
            })],
        );

        let mut unit_quaternion_instance_methods = HashMap::new();
        unit_quaternion_instance_methods.insert(
            "inv",
            vec![InstanceMethod::new(vec![], |val, _args| {
                Ok(Value::Quaternion(Arc::new(Mutex::new(
                    val.as_quaternion()?.inv(),
                ))))
            })],
        );

        structs.insert(
            "UnitQuaternion",
            Struct::new(
                unit_quaternion_struct_methods,
                unit_quaternion_instance_methods,
            ),
        );

        // Vector
        let mut vector_struct_methods = HashMap::new();
        vector_struct_methods.insert(
            "rand",
            vec![StructMethod::new(vec![Argument::new("n", "i64")], |args| {
                let mut rng = rand::rng();
                let vector =
                    DVector::from_vec((0..args[0].as_usize()?).map(|_| rng.random()).collect());
                Ok(Value::Vector(Arc::new(Mutex::new(vector))))
            })],
        );
        vector_struct_methods.insert(
            "randn",
            vec![StructMethod::new(vec![Argument::new("n", "i64")], |args| {
                let size = args[0].as_usize()?;

                let mut rng = rand::rng();
                let normal = Normal::new(0.0, 1.0).unwrap();
                let vector =
                    DVector::from_vec((0..size).map(|_| normal.sample(&mut rng)).collect());
                Ok(Value::Vector(Arc::new(Mutex::new(vector))))
            })],
        );

        let vector_instance_methods = HashMap::new();
        structs.insert(
            "Vector",
            Struct::new(vector_struct_methods, vector_instance_methods),
        );

        // Standalone Functions
        let mut functions = HashMap::new();
        functions.insert(
            "animate",
            vec![FunctionMethod::new(vec![], |_args| {
                Ok(Value::Event(Event::Animate))
            })],
        );
        functions.insert(
            "close_all_figures",
            vec![FunctionMethod::new(vec![], |_args| {
                Ok(Value::Event(Event::CloseAllFigures))
            })],
        );
        functions.insert(
            "figure",
            vec![FunctionMethod::new(vec![], |_args| {
                Ok(Value::Event(Event::NewFigure))
            })],
        );

        Self { structs, functions }
    }
}

impl Registry {
    pub fn eval_struct_method(
        &self,
        struct_name: &str,
        method_name: &str,
        mut args: Vec<Value>,
    ) -> Result<Value, RegistryErrors> {
        // Get the struct from the registry, or return an error if not found
        let struc = self
            .structs
            .get(struct_name)
            .ok_or_else(|| RegistryErrors::StructNotFound(struct_name.to_string()))?;

        // Get the list of possible method signatures
        let method_overloads = struc.struct_methods.get(method_name).ok_or_else(|| {
            RegistryErrors::MethodNotFound(struct_name.to_string(), method_name.to_string())
        })?;

        // Try each possible overload
        for method in method_overloads {
            // Check if argument lengths match
            if method.args.len() != args.len() {
                continue; // mismatch, try next overload
            }

            // Check if each argument type matches
            let mut signature_matches = true;
            for (required, actual) in method.args.iter().zip(args.iter_mut()) {
                if required.type_name != actual.to_string() {
                    // try to convert ints to float if possible, need better conversion logic
                    if required.type_name == "f64".to_string() && actual.to_string() == "i64" {
                        *actual = Value::f64(actual.as_f64()?)
                    } else {
                        signature_matches = false;
                        break;
                    }
                }
            }

            if !signature_matches {
                continue; // mismatch, try next overload
            }

            // If we get here, we've found a matching signature. Call the implementation.
            return (method.implementation)(args);
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

    pub fn eval_instance_method(
        &self,
        instance: Value,
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
        let method_overloads = struc.instance_methods.get(method_name).ok_or_else(|| {
            RegistryErrors::MethodNotFound(struct_name.to_string(), method_name.to_string())
        })?;
        // Try each possible overload
        for method in method_overloads {
            // Check if argument lengths match
            if method.args.len() != args.len() {
                continue; // mismatch, try next overload
            }

            // Check if each argument type matches
            let mut signature_matches = true;
            for (required, actual) in method.args.iter().zip(args.iter()) {
                if required.type_name != actual.to_string() && required.type_name != "Value" {
                    signature_matches = false;
                    break;
                }
            }

            if !signature_matches {
                continue; // mismatch, try next overload
            }

            // If we get here, we've found a matching signature. Call the implementation.
            return (method.implementation)(instance, args);
        }

        // If we exhaust all overloads without finding a match:
        Err(RegistryErrors::MethodNotFound(
            struct_name.to_string(),
            method_name.to_string(),
        ))
    }

    pub fn eval_function(
        &self,
        function_name: &str,
        args: Vec<Value>,
    ) -> Result<Value, RegistryErrors> {
        // Get the struct from the registry, or return an error if not found
        let method_overloads = self
            .functions
            .get(function_name)
            .ok_or_else(|| RegistryErrors::FunctionNotFound(function_name.to_string()))?;

        // Try each possible overload
        for method in method_overloads {
            // Check if argument lengths match
            if method.args.len() != args.len() {
                continue; // mismatch, try next overload
            }

            // Check if each argument type matches
            let mut signature_matches = true;
            for (required, actual) in method.args.iter().zip(args.iter()) {
                if required.type_name != actual.to_string() {
                    signature_matches = false;
                    break;
                }
            }

            if !signature_matches {
                continue; // mismatch, try next overload
            }

            // If we get here, we've found a matching signature. Call the implementation.
            return (method.implementation)(args);
        }
        // If we exhaust all overloads without finding a match:
        Err(RegistryErrors::FunctionNotFound(function_name.to_string()))
    }
}
