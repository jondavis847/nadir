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
    structs: HashMap<&'static str, Vec<&'static str>>,
}

impl Default for Registry {
    fn default() -> Self {
        let mut structs = HashMap::new();

        // Matrix
        structs.insert("Matrix", vec!["rand", "randn"]);

        // Quaternion
        structs.insert("Quaternion", vec!["new", "rand"]);

        // Time
        structs.insert("Time", vec!["new", "now"]);

        // UnitQuaternion
        structs.insert("UnitQuaternion", vec!["new", "rand"]);

        // Vector
        structs.insert("Vector", vec!["rand", "randn"]);

        Self { structs }
    }
}

impl Registry {
    pub fn eval_struct_method(
        &self,
        struct_name: &str,
        method_name: &str,
        args: Vec<Value>,
    ) -> Result<Value, RegistryErrors> {
        if let Some(methods) = self.structs.get(struct_name) {
            if methods.contains(&method_name) {
                match (struct_name, method_name) {
                    ("Matrix", "rand") => {
                        let (rows, cols) = match args.len() {
                            1 => {
                                let a0 = args[0].as_usize()?;
                                (a0, a0)
                            }
                            2 => (args[0].as_usize()?, args[1].as_usize()?),
                            _ => {
                                return Err(RegistryErrors::NumberOfArgs(
                                    method_name.to_string(),
                                    "1|2".to_string(),
                                    args.len().to_string(),
                                ));
                            }
                        };

                        let mut rng = rand::thread_rng();
                        let data: Vec<f64> = (0..(rows * cols)).map(|_| rng.gen::<f64>()).collect();
                        let matrix = DMatrix::from_vec(rows, cols, data);
                        Ok(Value::Matrix(Box::new(matrix)))
                    }
                    ("Matrix", "randn") => {
                        if args.len() != 2 {
                            return Err(RegistryErrors::NumberOfArgs(
                                method_name.to_string(),
                                "2".to_string(),
                                args.len().to_string(),
                            ));
                        }

                        let rows = args[0].as_usize()?; // Number of rows
                        let cols = args[1].as_usize()?; // Number of columns

                        let mut rng = thread_rng();
                        let normal = Normal::new(0.0, 1.0).unwrap(); // Standard normal (μ=0, σ=1)

                        let data: Vec<f64> = (0..(rows * cols))
                            .map(|_| normal.sample(&mut rng))
                            .collect();

                        let matrix = DMatrix::from_vec(rows, cols, data);

                        Ok(Value::Matrix(Box::new(matrix)))
                    }

                    ("Quaternion", "new") => {
                        if args.len() != 4 {
                            return Err(RegistryErrors::NumberOfArgs(
                                method_name.to_string(),
                                "4".to_string(),
                                args.len().to_string(),
                            ));
                        }
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
                    ("Quaternion", "rand") => {
                        if args.len() != 0 {
                            return Err(RegistryErrors::NumberOfArgs(
                                method_name.to_string(),
                                "0".to_string(),
                                args.len().to_string(),
                            ));
                        }

                        Ok(Value::Quaternion(Box::new(Quaternion::rand())))
                    }
                    ("Time", "now") => {
                        if args.len() > 0 {
                            return Err(RegistryErrors::NumberOfArgs(
                                method_name.to_string(),
                                "0".to_string(),
                                args.len().to_string(),
                            ));
                        }
                        Ok(Value::Time(Box::new(Time::now()?)))
                    }
                    ("UnitQuaternion", "new") => {
                        if args.len() != 4 {
                            return Err(RegistryErrors::NumberOfArgs(
                                method_name.to_string(),
                                "4".to_string(),
                                args.len().to_string(),
                            ));
                        }
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
                        if args.len() != 0 {
                            return Err(RegistryErrors::NumberOfArgs(
                                method_name.to_string(),
                                "0".to_string(),
                                args.len().to_string(),
                            ));
                        }

                        Ok(Value::UnitQuaternion(Box::new(UnitQuaternion::rand())))
                    }
                    // called as Vector::rand(type,length)
                    ("Vector", "rand") => {
                        if args.len() != 1 {
                            return Err(RegistryErrors::NumberOfArgs(
                                method_name.to_string(),
                                "1".to_string(),
                                args.len().to_string(),
                            ));
                        }
                        let mut rng = rand::thread_rng();
                        let vector = DVector::from_vec(
                            (0..args[0].as_usize()?).map(|_| rng.gen::<f64>()).collect(),
                        );
                        Ok(Value::Vector(Box::new(vector)))
                    }
                    ("Vector", "randn") => {
                        if args.len() != 1 {
                            return Err(RegistryErrors::NumberOfArgs(
                                method_name.to_string(),
                                "1".to_string(),
                                args.len().to_string(),
                            ));
                        }

                        let size = args[0].as_usize()?; // Get vector length

                        let mut rng = thread_rng();
                        let normal = Normal::new(0.0, 1.0).unwrap(); // Standard normal distribution (μ=0, σ=1)

                        let vector =
                            DVector::from_vec((0..size).map(|_| normal.sample(&mut rng)).collect());

                        Ok(Value::Vector(Box::new(vector)))
                    }
                    _ => unreachable!(
                        "we checked if struct and method exist, if you got here there's a bug"
                    ),
                }
            } else {
                return Err(RegistryErrors::MethodNotFound(
                    struct_name.to_string(),
                    method_name.to_string(),
                ));
            }
        } else {
            return Err(RegistryErrors::StructNotFound(struct_name.to_string()));
        }
    }

    pub fn get_structs(&self) -> Vec<&'static str> {
        self.structs.keys().into_iter().cloned().collect()
    }

    pub fn get_struct_methods(&self, struc: &str) -> Result<Vec<&'static str>, RegistryErrors> {
        if let Some(methods) = self.structs.get(struc) {
            Ok(methods.clone())
        } else {
            Err(RegistryErrors::StructNotFound(struc.to_string()))
        }
    }
}
