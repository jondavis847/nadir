use csv::ReaderBuilder;
use inquire::Select;
use nalgebra::{DMatrix, DVector};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use rotations::prelude::{Quaternion, QuaternionErrors, UnitQuaternion};
use std::{
    collections::HashMap,
    ffi::OsString,
    fs::{self, File},
    io::BufReader,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};
use thiserror::Error;
use time::TimeErrors;

use crate::value::{Event, Linspace, LinspaceErrors, Map, Value, ValueErrors};

#[derive(Debug, Error)]
pub enum RegistryErrors {
    #[error("command '{0}' not found")]
    CommandNotFound(String),
    #[error("function '{0}' not found")]
    FunctionNotFound(String),
    #[error("{0}")]
    IoError(#[from] std::io::Error),
    #[error("{0}")]
    LinspaceErrors(#[from] LinspaceErrors),
    #[error("method '{1}' not found for struct '{0}'")]
    MethodNotFound(String, String),
    #[error("path {0} could not canonicalize")]
    PathCouldntCanonicalize(String),
    #[error("path {0} does not exist")]
    PathDoesNotExist(String),
    #[error("path {0} is not a directory")]
    PathIsNotADir(String),
    #[error("{0}")]
    QuaternionErrors(#[from] QuaternionErrors),
    #[error("struct '{0}' not found")]
    StructNotFound(String),
    #[error("{0}")]
    TimeErrors(#[from] TimeErrors),
    #[error("{0}")]
    ValueErrors(#[from] ValueErrors),
    #[error("{0} should have {1} arguments")]
    WrongNumberArgs(String, String),
    #[error("{0}")]
    Error(#[from] Box<dyn std::error::Error>),
}

// Types for method implementations
type FunctionFn = fn(Vec<Value>) -> Result<Value, RegistryErrors>;
type StructMethodFn = fn(Vec<Value>, Arc<Mutex<PathBuf>>) -> Result<Value, RegistryErrors>;
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

#[derive(Debug)]
pub struct CommandMethod {
    pub implementation: CommandFn,
}
impl CommandMethod {
    fn new(implementation: CommandFn) -> Self {
        Self { implementation }
    }
}

type CommandFn = fn(Vec<String>, Arc<Mutex<PathBuf>>) -> Result<Value, RegistryErrors>;

pub struct Registry {
    pub structs: HashMap<&'static str, Struct>,
    pub functions: HashMap<&'static str, Vec<FunctionMethod>>,
    pub commands: HashMap<&'static str, CommandMethod>, // basically the same as functions, just different syntax
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
                |args, _pwd| {
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
            vec![StructMethod::new(vec![], |_args, _pwd| {
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
                StructMethod::new(vec![Argument::new("n", "i64")], |args, _pwd| {
                    let n = args[0].as_usize()?;
                    let mut rng = rand::rng();
                    let data: Vec<f64> = (0..(n * n)).map(|_| rng.random()).collect();
                    let matrix = DMatrix::from_vec(n, n, data);
                    Ok(Value::Matrix(Arc::new(Mutex::new(matrix))))
                }),
                StructMethod::new(
                    vec![Argument::new("m", "i64"), Argument::new("n", "i64")],
                    |args, _pwd| {
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
                StructMethod::new(vec![Argument::new("n", "i64")], |args, _pwd| {
                    let n = args[0].as_usize()?;
                    let mut rng = rand::rng();
                    let normal = Normal::new(0.0, 1.0 / 3.0).unwrap();
                    let data: Vec<f64> = (0..(n * n)).map(|_| normal.sample(&mut rng)).collect();
                    let matrix = DMatrix::from_vec(n, n, data);
                    Ok(Value::Matrix(Arc::new(Mutex::new(matrix))))
                }),
                StructMethod::new(
                    vec![Argument::new("m", "i64"), Argument::new("n", "i64")],
                    |args, _pwd| {
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
                |args, _pwd| {
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
            vec![StructMethod::new(vec![], |_, _pwd| {
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
                |args, _pwd| {
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
                |args, _pwd| {
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
            vec![StructMethod::new(vec![], |_args, _pwd| {
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
            vec![StructMethod::new(
                vec![Argument::new("n", "i64")],
                |args, _pwd| {
                    let mut rng = rand::rng();
                    let vector =
                        DVector::from_vec((0..args[0].as_usize()?).map(|_| rng.random()).collect());
                    Ok(Value::Vector(Arc::new(Mutex::new(vector))))
                },
            )],
        );
        vector_struct_methods.insert(
            "randn",
            vec![StructMethod::new(
                vec![Argument::new("n", "i64")],
                |args, _pwd| {
                    let size = args[0].as_usize()?;

                    let mut rng = rand::rng();
                    let normal = Normal::new(0.0, 1.0).unwrap();
                    let vector =
                        DVector::from_vec((0..size).map(|_| normal.sample(&mut rng)).collect());
                    Ok(Value::Vector(Arc::new(Mutex::new(vector))))
                },
            )],
        );
        vector_struct_methods.insert(
            "from_csv",
            vec![StructMethod::new(vec![], |_args, pwd| {
                let pwd = &*pwd.lock().unwrap();
                // Start the file selection loop
                let selected_file = navigate_and_select_file(pwd)?;

                // Load and process the selected file
                let file_ext = selected_file
                    .extension()
                    .and_then(|ext| ext.to_str())
                    .unwrap_or("");

                let v = match file_ext {
                    "csv" => load_csv_file(&selected_file)?,
                    "txt" => load_txt_file(&selected_file)?,
                    "42" => load_42_file(&selected_file)?,
                    _ => {
                        return Err(RegistryErrors::Error(
                            format!("Unsupported file extension: {}", file_ext).into(),
                        ));
                    }
                };
                Ok(v)
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

        // Commands like cd . etc,
        let mut commands = HashMap::new();
        commands.insert(
            "cd",
            CommandMethod::new(|args, path| {
                use std::fs;
                use std::path::{Path, PathBuf};

                if args.len() != 1 {
                    return Err(RegistryErrors::WrongNumberArgs(
                        "cd".to_string(),
                        "1".to_string(),
                    ));
                }
                let path = &mut *path.lock().unwrap();
                let input_path = &args[0];
                let new_path: PathBuf = if input_path == "~" {
                    // Handle home directory
                    dirs::home_dir().unwrap_or_else(|| PathBuf::from("/"))
                } else if input_path == ".." {
                    // Go up one directory
                    path.parent()
                        .map(|p| p.to_path_buf())
                        .unwrap_or_else(|| path.clone())
                } else if input_path == "." {
                    // Stay in current directory
                    path.clone()
                } else if Path::new(&input_path).is_absolute() {
                    // Absolute path
                    PathBuf::from(input_path)
                } else {
                    // Relative path
                    let mut new_path = path.clone();
                    new_path.push(input_path);
                    new_path
                };

                // Verify the directory exists
                if !new_path.exists() {
                    return Err(RegistryErrors::PathDoesNotExist(
                        new_path.to_string_lossy().to_string(),
                    ));
                }

                if !new_path.is_dir() {
                    return Err(RegistryErrors::PathIsNotADir(
                        new_path.to_string_lossy().to_string(),
                    ));
                }

                // Canonicalize the path to resolve any symlinks or relative components
                match fs::canonicalize(&new_path) {
                    Ok(canonical_path) => {
                        *path = canonical_path;
                        Ok(Value::None)
                    }
                    Err(_e) => Err(RegistryErrors::PathCouldntCanonicalize(
                        new_path.to_string_lossy().to_string(),
                    )),
                }
            }),
        );
        commands.insert(
            "pwd",
            CommandMethod::new(|_args, path| {
                let path = &*path.lock().unwrap();
                let mut path_string = path.to_string_lossy().to_string();
                if cfg!(windows) && path_string.starts_with(r"\\?\") {
                    path_string = path_string[4..].to_string();
                }
                println!("{path_string}");
                Ok(Value::None)
            }),
        );

        commands.insert(
            "ls",
            CommandMethod::new(|_args, path| {
                let mut entries = Vec::new();
                let path = &*path.lock().unwrap();
                // Collect directory entries
                for entry in fs::read_dir(path)? {
                    let entry = entry?;
                    let file_type = entry.file_type()?;
                    let file_name = entry.file_name().into_string().unwrap();

                    // No color formatting, just add separator for directories
                    let display_name = if file_type.is_dir() {
                        format!("{}{}", file_name, std::path::MAIN_SEPARATOR)
                    } else {
                        file_name
                    };

                    entries.push(display_name);
                }

                // Sort entries (optional)
                entries.sort();

                // Get terminal width
                let terminal_width = match term_size::dimensions() {
                    Some((width, _)) => width,
                    None => 80, // Default if terminal size can't be determined
                };

                // Display in columns
                let formatted_output = display_in_columns(&entries, terminal_width);
                print!("{}", formatted_output);

                Ok(Value::None)
            }),
        );

        Self {
            structs,
            functions,
            commands,
        }
    }
}

impl Registry {
    pub fn eval_struct_method(
        &self,
        struct_name: &str,
        method_name: &str,
        mut args: Vec<Value>,
        pwd: Arc<Mutex<PathBuf>>,
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
            return (method.implementation)(args, pwd);
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

    pub fn eval_command(
        &self,
        cmd_name: &str,
        args: Vec<String>,
        pwd: Arc<Mutex<PathBuf>>,
    ) -> Result<Value, RegistryErrors> {
        // Get the struct from the registry, or return an error if not found
        let method = self
            .commands
            .get(cmd_name)
            .ok_or_else(|| RegistryErrors::CommandNotFound(cmd_name.to_string()))?;

        (method.implementation)(args, pwd)
    }
}

fn display_in_columns(entries: &[String], terminal_width: usize) -> String {
    if entries.is_empty() {
        return String::new();
    }

    // Find the maximum entry length for column width calculation
    let max_len = entries.iter().map(|entry| entry.len()).max().unwrap_or(0);

    // Add padding between columns
    let column_width = max_len + 2;

    // Calculate how many columns can fit in the terminal
    let num_columns = if column_width > 0 {
        std::cmp::max(1, terminal_width / column_width)
    } else {
        1
    };

    // Calculate how many rows we need
    let num_rows = (entries.len() + num_columns - 1) / num_columns;

    let mut result = String::new();

    // Format in columns
    for row in 0..num_rows {
        for col in 0..num_columns {
            let index = row + col * num_rows;
            if index < entries.len() {
                let entry = &entries[index];
                // Add padding to align columns
                result.push_str(&format!("{:<width$}", entry, width = column_width));
            }
        }
        result.push('\n');
    }

    result
}

// Function to navigate directories and select a file
fn navigate_and_select_file(start_dir: &Path) -> Result<PathBuf, Box<dyn std::error::Error>> {
    let mut current_dir = start_dir.to_path_buf();

    loop {
        // Read directory contents
        let entries = fs::read_dir(&current_dir)?;

        // Filter and collect valid entries
        let mut valid_entries = Vec::new();
        let parent_dir = current_dir.parent().map(|p| p.to_path_buf());

        // Add parent directory option if not at root
        if parent_dir.is_some() {
            valid_entries.push(("..".to_string(), None));
        }

        // Add directories and valid files
        for entry in entries {
            let entry = entry?;
            let path = entry.path();
            let file_type = entry.file_type()?;

            let name = path
                .file_name()
                .and_then(|n| n.to_str())
                .unwrap_or("[Invalid Name]")
                .to_string();

            if file_type.is_dir() {
                valid_entries.push((format!("{}/", name), Some(path)));
            } else if file_type.is_file() {
                // Check if file has a valid extension
                if let Some(ext) = path.extension().and_then(|e| e.to_str()) {
                    if ["csv", "txt", "42"].contains(&ext) {
                        valid_entries.push((name, Some(path)));
                    }
                }
            }
        }

        // Sort entries with parent directory first, files second, and directories last
        valid_entries.sort_by(|(name_a, _), (name_b, _)| {
            // Special case: Parent directory (..) always comes first
            if name_a == ".." {
                return std::cmp::Ordering::Less;
            }
            if name_b == ".." {
                return std::cmp::Ordering::Greater;
            }

            let is_dir_a = name_a.ends_with('/');
            let is_dir_b = name_b.ends_with('/');

            match (is_dir_a, is_dir_b) {
                // If one is a directory and one is a file, put files before directories
                (true, false) => std::cmp::Ordering::Greater, // Changed from Less to Greater
                (false, true) => std::cmp::Ordering::Less,    // Changed from Greater to Less

                // If both are the same type, sort alphabetically
                _ => name_a.cmp(name_b),
            }
        });

        // Extract just the display names for selection
        let options: Vec<&str> = valid_entries
            .iter()
            .map(|(name, _)| name.as_str())
            .collect();

        // No valid options? Return an error
        if options.is_empty() {
            return Err("No valid files or directories found.".to_string().into());
        }

        // Prompt user to select an entry
        let selection = Select::new("Select a file or directory:", options)
            .prompt()
            .map_err(|e| format!("Selection failed: {}", e))?;

        // Find the selected entry directly from valid_entries
        let selected_idx = valid_entries
            .iter()
            .position(|(name, _)| name.as_str() == selection)
            .ok_or("Selection not found")?;

        // Handle the selection
        match &valid_entries[selected_idx] {
            (name, None) if name == ".." => {
                // Go up to parent directory
                if let Some(parent) = parent_dir {
                    current_dir = parent;
                }
            }
            (_, Some(path)) if path.is_dir() => {
                // Navigate into the selected directory
                current_dir = path.clone();
            }
            (_, Some(path)) => {
                // Selected a file, return it
                return Ok(path.clone());
            }
            _ => {
                return Err("Invalid selection".to_string().into());
            }
        }
    }
}

// Function to load CSV files
fn load_csv_file(file_path: &Path) -> Result<Value, Box<dyn std::error::Error>> {
    // Open the file
    let file = File::open(file_path)?;
    let reader = BufReader::new(file);

    // Create a CSV reader
    let mut csv_reader = ReaderBuilder::new()
        .has_headers(true)
        .flexible(true)
        .from_reader(reader);

    // Check if the file has headers
    let headers = csv_reader.headers().map(|h| h.clone()).ok();

    let column_options = if let Some(headers) = &headers {
        // File has headers - offer them as options
        headers
            .iter()
            .enumerate()
            .map(|(i, h)| format!("{}: {}", i + 1, h))
            .collect::<Vec<_>>()
    } else {
        let record = csv_reader
            .records()
            .next()
            .ok_or_else(|| "CSV file appears to be empty".to_string())?
            .map_err(|e| format!("Failed to read CSV: {}", e))?;

        (1..=record.len())
            .map(|i| format!("Column {}", i))
            .collect()
    };

    // Prompt user to select a column
    let column_selection = Select::new("Select a column to load:", column_options)
        .prompt()
        .map_err(|e| format!("Selection failed: {}", e))?;

    // Extract the column index from the selection string
    let column_idx = if column_selection.starts_with("Column ") {
        // Parse from "Column X"
        column_selection[7..]
            .parse::<usize>()
            .map_err(|_| "Failed to parse column index".to_string())?
            - 1
    } else {
        // Parse from "X: Header"
        column_selection
            .split(':')
            .next()
            .and_then(|s| s.parse::<usize>().ok())
            .map(|i| i - 1)
            .ok_or("Failed to parse column index".to_string())?
    };

    // Reset and reopen the CSV reader
    let file = File::open(file_path).map_err(|e| format!("Failed to reopen file: {}", e))?;
    let reader = BufReader::new(file);
    let mut csv_reader = ReaderBuilder::new()
        .has_headers(headers.is_some())
        .from_reader(reader);

    // Skip header row if needed
    if headers.is_some() {
        let _ = csv_reader.headers();
    }

    // Read the selected column into a vector
    let mut values = Vec::new();

    for result in csv_reader.records() {
        let record = result.map_err(|e| format!("Failed to read record: {}", e))?;

        if column_idx < record.len() {
            // Try to parse the value as a floating point number
            let value_str = record.get(column_idx).unwrap_or_default();
            values.push(value_str.parse::<f64>()?);
        } else {
            return Err(format!("Column index {} out of bounds", column_idx + 1).into());
        }
    }

    // Create DVector from the values
    let vector = DVector::from_vec(values);
    Ok(Value::Vector(Arc::new(Mutex::new(vector))))
}

// Function to load TXT files
fn load_txt_file(file_path: &Path) -> Result<Value, Box<dyn std::error::Error>> {
    let contents = fs::read_to_string(file_path)
        .map_err(|e| format!("Failed to read file {}: {}", file_path.display(), e))?;

    // Split the content by lines and/or whitespace
    let values: Result<Vec<f64>, _> = contents
        .lines()
        .flat_map(|line| line.split_whitespace())
        .map(|s| {
            s.parse::<f64>()
                .map_err(|_| format!("Invalid number: '{}'", s))
        })
        .collect();

    let values = values?;

    if values.is_empty() {
        return Err("File contains no valid numeric values".to_string().into());
    }

    // Create DVector from the values
    let vector = DVector::from_vec(values);
    Ok(Value::Vector(Arc::new(Mutex::new(vector))))
}

// Function to load .42 files (assuming this is a custom format)
fn load_42_file(file_path: &PathBuf) -> Result<Value, Box<dyn std::error::Error>> {
    let contents = fs::read_to_string(file_path)
        .map_err(|e| format!("Failed to read file {}: {}", file_path.display(), e))?;

    // Parse your custom .42 file format here
    // This is just a placeholder implementation assuming .42 files are similar to txt files
    let values: Result<Vec<f64>, _> = contents
        .lines()
        .filter(|line| !line.trim().starts_with('#')) // Skip comment lines
        .flat_map(|line| line.split_whitespace())
        .map(|s| {
            s.parse::<f64>()
                .map_err(|_| format!("Invalid number: '{}'", s))
        })
        .collect();

    let values = values?;

    if values.is_empty() {
        return Err("File contains no valid numeric values".to_string().into());
    }

    // Create DVector from the values
    let vector = DVector::from_vec(values);
    Ok(Value::Vector(Arc::new(Mutex::new(vector))))
}
