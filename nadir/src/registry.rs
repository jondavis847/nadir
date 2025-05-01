use csv::{ReaderBuilder, StringRecord, Trim};
use inquire::{MultiSelect, Select};
use nalgebra::{DMatrix, DVector};
use rand::Rng;
use rand_distr::{Distribution, Normal};
use rotations::prelude::{Quaternion, QuaternionErrors, UnitQuaternion};
use std::{
    collections::HashMap,
    fs::{self, File},
    io::BufReader,
    path::{Path, PathBuf},
    sync::{Arc, Mutex},
};
use thiserror::Error;
use time::TimeErrors;

use crate::{
    plotting::{PlotErrors, figure::Figure, line::Line, series::Series},
    value::{Event, Linspace, LinspaceErrors, Map, Value, ValueErrors},
};

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
    PlotErrors(#[from] PlotErrors),
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
type FunctionFn = fn(Vec<Value>, Arc<Mutex<PathBuf>>) -> Result<Value, RegistryErrors>;
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

        // Axes
        let axes_struct_methods = HashMap::new();

        let mut axes_instance_methods = HashMap::new();
        axes_instance_methods.insert(
            "add_line",
            vec![InstanceMethod::new(
                vec![Argument::new("line", "Line")],
                |val, args| {
                    let line = args[0].as_line()?;
                    let axes_guard = val.as_axes()?;
                    let mut axes = axes_guard.lock().unwrap();

                    axes.add_line(line);
                    Ok(Value::Event(Event::ClearCache(
                        axes.get_figure_id().unwrap(),
                    )))
                },
            )],
        );

        structs.insert(
            "Axes",
            Struct::new(axes_struct_methods, axes_instance_methods),
        );

        // Figure
        let mut figure_struct_methods = HashMap::new();
        figure_struct_methods.insert(
            "new",
            vec![StructMethod::new(vec![], |_args, _pwd| {
                let figure = Arc::new(Mutex::new(Figure::new()));
                Ok(Value::Event(Event::NewFigure(figure)))
            })],
        );
        figure_struct_methods.insert(
            "from_file",
            vec![StructMethod::new(vec![], |_args, pwd| {
                let pwd = pwd.lock().unwrap();
                let file = navigate_and_select_file(&pwd)?;
                let (xname, x) = load_single_column(&file)?;
                let file = navigate_and_select_file(&pwd)?;
                let ydata = load_multiple_columns(&file)?;

                let mut figure = Figure::new();
                let axes = figure.get_axes(0)?;
                let axes = &mut *axes.lock().unwrap();

                let file_name = file.file_stem();

                for (yname, y) in ydata {
                    let mut series = Series::new(&x, &y)?;
                    series.set_x_name(xname.clone());
                    let yname = if let Some(file_name) = &file_name {
                        format!("{}::{}", file_name.to_string_lossy(), yname)
                    } else {
                        yname
                    };
                    series.set_y_name(yname);
                    let l = Arc::new(Mutex::new(Line::new(series)));
                    axes.add_line(l);
                }

                Ok(Value::Event(Event::NewFigure(Arc::new(Mutex::new(figure)))))
            })],
        );
        let mut figure_instance_methods = HashMap::new();
        figure_instance_methods.insert(
            "add_axes",
            vec![InstanceMethod::new(
                vec![Argument::new("row", "i64"), Argument::new("col", "i64")],
                |val, args| {
                    let row = args[0].as_usize()?;
                    let col = args[1].as_usize()?;
                    let plot = val.as_figure()?;
                    let plot = &mut *plot.lock().unwrap();
                    if let Some(id) = plot.get_id() {
                        plot.add_axes(row, col);
                        Ok(Value::Event(Event::ClearCache(id)))
                    } else {
                        return Err(RegistryErrors::Error(
                            "Figure ID not set. Ensure the figure is created first.".into(),
                        ));
                    }
                },
            )],
        );
        figure_instance_methods.insert(
            "get_axes",
            vec![InstanceMethod::new(
                vec![Argument::new("index", "i64")],
                |val, args| {
                    let i = args[0].as_usize()?;
                    let plot = val.as_figure()?;
                    let plot = &mut *plot.lock().unwrap();
                    let axes = match plot.get_axes(i) {
                        Ok(axes) => axes,
                        Err(e) => return Err(RegistryErrors::Error(e.into())),
                    };
                    Ok(Value::Axes(axes))
                },
            )],
        );

        structs.insert(
            "Figure",
            Struct::new(figure_struct_methods, figure_instance_methods),
        );

        // Line
        let mut line_struct_methods = HashMap::new();
        line_struct_methods.insert(
            "new",
            vec![StructMethod::new(
                vec![
                    Argument::new("x_data", "Vector"),
                    Argument::new("y_data", "Vector"),
                ],
                |args, _pwd| {
                    let x_data = args[0].as_vector()?;
                    let x_data = &*x_data.lock().unwrap();
                    let y_data = args[1].as_vector()?;
                    let y_data = &*y_data.lock().unwrap();
                    let series = match Series::new(x_data, y_data) {
                        Ok(series) => series,
                        Err(e) => return Err(RegistryErrors::Error(e.into())),
                    };
                    let line = Line::new(series);
                    Ok(Value::Line(Arc::new(Mutex::new(line))))
                },
            )],
        );

        line_struct_methods.insert(
            "from_file",
            vec![StructMethod::new(vec![], |_args, pwd| {
                let pwd = pwd.lock().unwrap();
                let file = navigate_and_select_file(&pwd)?;
                let (xname, x) = load_single_column(&file)?;
                let file = navigate_and_select_file(&pwd)?;
                let (yname, y) = load_single_column(&file)?;
                let mut series = Series::new(&x, &y)?;
                series.set_x_name(xname);
                series.set_y_name(yname);
                let l = Arc::new(Mutex::new(Line::new(series)));
                Ok(Value::Line(l))
            })],
        );
        let line_instance_methods = HashMap::new();
        structs.insert(
            "Line",
            Struct::new(line_struct_methods, line_instance_methods),
        );

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

        structs.insert(
            "Quaternion",
            Struct::new(quaternion_struct_methods, quaternion_instance_methods),
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
            "from_file",
            vec![StructMethod::new(vec![], |_args, pwd| {
                let pwd = pwd.lock().unwrap();
                let file = navigate_and_select_file(&pwd)?;
                let (_, v) = load_single_column(&file)?;
                Ok(Value::Vector(Arc::new(Mutex::new(v))))
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
            vec![FunctionMethod::new(vec![], |_args, pwd| {
                Ok(Value::Event(Event::Animate(pwd)))
            })],
        );
        functions.insert(
            "close_all_figures",
            vec![FunctionMethod::new(vec![], |_args, _pwd| {
                Ok(Value::Event(Event::CloseAllFigures))
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
                if required.type_name != actual.type_check() {
                    // try to convert ints to float if possible, need better conversion logic
                    if required.type_name == "f64".to_string() && actual.type_check() == "i64" {
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
        pwd: Arc<Mutex<PathBuf>>,
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
            return (method.implementation)(args, pwd);
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
                (true, false) => std::cmp::Ordering::Greater,
                (false, true) => std::cmp::Ordering::Less,

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

/// Parse column index from a formatted column selection string
fn parse_column_index(selection: &str) -> Result<usize, String> {
    if let Some(pos) = selection.find(':') {
        selection[..pos]
            .trim()
            .parse::<usize>()
            .map_err(|_| "Failed to parse column index".to_string())
            .map(|idx| idx - 1)
    } else if let Some(start) = selection.find('[') {
        if let Some(end) = selection.find(']') {
            selection[start + 1..end]
                .parse::<usize>()
                .map_err(|_| "Failed to parse column index".to_string())
                .map(|idx| idx - 1)
        } else {
            Err("Invalid column format, missing closing bracket".to_string())
        }
    } else {
        Err("Failed to parse column selection".to_string())
    }
}

/// Smart file utility function that adapts to various file formats
fn smart_file_utils(
    file_path: &Path,
) -> Result<(Vec<String>, Option<StringRecord>, String, u8), Box<dyn std::error::Error>> {
    // Get file name for column names
    let file_stem = file_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("file")
        .to_string();

    // Filter out comments for any file that might have them
    let content = fs::read_to_string(file_path)?;
    let filtered_content: String = content
        .lines()
        .filter(|line| !line.trim().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");

    if filtered_content.trim().is_empty() {
        return Err("File is empty or contains only comments".into());
    }

    // Detect the most likely delimiter
    let possible_delimiters = [b',', b'\t', b';', b' ', b'|'];
    let mut best_delimiter = b','; // Default
    let mut max_consistency = 0;

    for &delimiter in &possible_delimiters {
        let reader = ReaderBuilder::new()
            .delimiter(delimiter)
            .has_headers(false)
            .flexible(true)
            .from_reader(filtered_content.as_bytes());

        // Check field count consistency across rows
        let records: Vec<csv::StringRecord> =
            reader.into_records().filter_map(Result::ok).collect();
        if records.is_empty() {
            continue;
        }

        // Count occurrences of each field length
        let mut field_counts = HashMap::new();
        for record in &records {
            let count = field_counts.entry(record.len()).or_insert(0);
            *count += 1;
        }

        let (_, consistency) = field_counts
            .iter()
            .max_by_key(|(_, count)| *count) // Remove the & symbol here
            .unwrap_or((&0, &0));

        if *consistency > max_consistency && records[0].len() > 1 {
            max_consistency = *consistency;
            best_delimiter = delimiter;
        }
    }

    // Now check if the file likely has headers
    let mut reader = ReaderBuilder::new()
        .delimiter(best_delimiter)
        .has_headers(false)
        .flexible(true)
        .from_reader(filtered_content.as_bytes());

    let records: Vec<csv::StringRecord> = reader.records().take(5).filter_map(Result::ok).collect();
    if records.len() < 2 {
        // Not enough data to determine headers, assume no headers
        let column_options: Vec<String> = if !records.is_empty() {
            (1..=records[0].len())
                .map(|i| format!("{}[{}]", file_stem, i))
                .collect()
        } else {
            Vec::new()
        };

        return Ok((column_options, None, file_stem, best_delimiter));
    }

    // Heuristic to detect headers: first row is more likely to contain text while
    // subsequent rows are more likely to contain numbers
    let first_row = &records[0];

    // Count how many fields in the first row look like headers
    let mut header_like_fields = 0;
    for field in first_row.iter() {
        // Headers often contain text that can't be parsed as numbers
        // or may have special patterns like brackets or underscores
        if field.parse::<f64>().is_err() || field.contains('_') || field.contains('[') {
            header_like_fields += 1;
        }
    }

    // Check if any subsequent row is mostly numeric
    let mut any_numeric_row = false;
    for row in &records[1..] {
        let mut numeric_fields = 0;
        for field in row.iter() {
            if field.parse::<f64>().is_ok() {
                numeric_fields += 1;
            }
        }

        // If most fields in a row are numeric, and first row has header-like fields
        if numeric_fields > row.len() / 2 && header_like_fields > 0 {
            any_numeric_row = true;
            break;
        }
    }

    // If first row has any header indicators and subsequent rows have numeric content
    let headers_likely = header_like_fields > 0 && any_numeric_row;

    // Generate column options based on our detection
    let (column_options, headers) = if headers_likely {
        // Use first row as headers
        let headers = Some(first_row.clone());

        let options = first_row
            .iter()
            .enumerate()
            .map(|(i, h)| format!("{}: {}", i + 1, h))
            .collect::<Vec<_>>();

        (options, headers)
    } else {
        // No headers, use column indices
        let options = (1..=first_row.len())
            .map(|i| format!("{}[{}]", file_stem, i))
            .collect::<Vec<_>>();

        (options, None)
    };

    Ok((column_options, headers, file_stem, best_delimiter))
}

/// Function to load a single column from any file type
fn load_single_column(
    file_path: &Path,
) -> Result<(String, DVector<f64>), Box<dyn std::error::Error>> {
    // Smart detection of file format
    let (column_options, headers, file_stem, delimiter) = smart_file_utils(file_path)?;

    if column_options.is_empty() {
        return Err("No data columns detected in file".into());
    }

    // Select a single column
    let column_selection = Select::new("Select a column to load:", column_options)
        .prompt()
        .map_err(|e| format!("Selection failed: {}", e))?;

    // Parse column index
    let column_idx = parse_column_index(&column_selection)?;

    // Get column name
    let header_name = if let Some(headers) = &headers {
        headers
            .get(column_idx)
            .map(|h| h.to_string())
            .unwrap_or_else(|| format!("{}[{}]", file_stem, column_idx + 1))
    } else {
        format!("{}[{}]", file_stem, column_idx + 1)
    };

    // Filter content to remove comments
    let content = fs::read_to_string(file_path)?;
    let filtered_content: String = content
        .lines()
        .filter(|line| !line.trim().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");

    // Create reader with detected settings
    let mut csv_reader = ReaderBuilder::new()
        .delimiter(delimiter)
        .has_headers(headers.is_some())
        .flexible(true)
        .trim(Trim::All)
        .from_reader(filtered_content.as_bytes());

    if headers.is_some() {
        let _ = csv_reader.headers();
    }

    // Read the data
    let mut values = Vec::new();
    for result in csv_reader.records() {
        let record = result.map_err(|e| format!("Failed to read record: {}", e))?;

        if column_idx < record.len() {
            let value_str = record.get(column_idx).unwrap_or_default();
            if !value_str.trim().is_empty() {
                let value = value_str.parse::<f64>().map_err(|_| {
                    format!(
                        "Failed to parse '{}' as f64 in column {}",
                        value_str,
                        column_idx + 1
                    )
                })?;
                values.push(value);
            }
        } else {
            return Err(format!("Column index {} out of bounds", column_idx + 1).into());
        }
    }

    if values.is_empty() {
        return Err(format!("No valid numeric data found for column {}", column_idx + 1).into());
    }

    Ok((header_name, DVector::from_vec(values)))
}

/// Unified function to load multiple columns from any file type
fn load_multiple_columns(
    file_path: &Path,
) -> Result<Vec<(String, DVector<f64>)>, Box<dyn std::error::Error>> {
    // Smart detection of file format
    let (column_options, headers, file_stem, delimiter) = smart_file_utils(file_path)?;

    if column_options.is_empty() {
        return Err("No data columns detected in file".into());
    }

    // Select columns
    let selected_columns = MultiSelect::new("Select columns to load:", column_options)
        .prompt()
        .map_err(|e| format!("Selection failed: {}", e))?;

    if selected_columns.is_empty() {
        return Err("No columns were selected".to_string().into());
    }

    // Parse column indices
    let column_indices: Vec<usize> = selected_columns
        .iter()
        .map(|selection| parse_column_index(selection))
        .collect::<Result<Vec<usize>, String>>()?;

    // Get column names
    let column_names: Vec<String> = column_indices
        .iter()
        .map(|&idx| {
            if let Some(headers) = &headers {
                headers
                    .get(idx)
                    .map(|h| h.to_string())
                    .unwrap_or_else(|| format!("{}[{}]", file_stem, idx + 1))
            } else {
                format!("{}[{}]", file_stem, idx + 1)
            }
        })
        .collect();

    // Filter content to remove comments
    let content = fs::read_to_string(file_path)?;
    let filtered_content: String = content
        .lines()
        .filter(|line| !line.trim().starts_with('#'))
        .collect::<Vec<_>>()
        .join("\n");

    // Create reader with detected settings
    let mut csv_reader = ReaderBuilder::new()
        .delimiter(delimiter)
        .has_headers(headers.is_some())
        .flexible(true)
        .trim(Trim::All)
        .from_reader(filtered_content.as_bytes());

    if headers.is_some() {
        let _ = csv_reader.headers();
    }

    // Create vectors to hold data for each selected column
    let mut column_data: Vec<Vec<f64>> = vec![Vec::new(); column_indices.len()];

    // Read all records and extract selected columns
    for result in csv_reader.records() {
        let record = result.map_err(|e| format!("Failed to read record: {}", e))?;

        for (vec_idx, &col_idx) in column_indices.iter().enumerate() {
            if col_idx < record.len() {
                let value_str = record.get(col_idx).unwrap_or_default();
                if !value_str.trim().is_empty() {
                    let value = value_str.parse::<f64>().map_err(|_| {
                        format!(
                            "Failed to parse value '{}' as f64 in column {}",
                            value_str,
                            col_idx + 1
                        )
                    })?;
                    column_data[vec_idx].push(value);
                }
            }
        }
    }

    // Validate data
    for (idx, data) in column_data.iter().enumerate() {
        if data.is_empty() {
            return Err(format!(
                "No valid numeric data found for column {}",
                column_indices[idx] + 1
            )
            .into());
        }
    }

    // Create result vector of (name, data) pairs
    let result: Vec<(String, DVector<f64>)> = column_names
        .into_iter()
        .zip(column_data.into_iter().map(DVector::from_vec))
        .collect();

    Ok(result)
}
