use ansi_term::Colour;
use nalgebra::{DMatrix, DVector};
use rotations::prelude::{Quaternion, UnitQuaternion};
use rustyline::error::ReadlineError;
use rustyline::{CompletionType, Config, EditMode, Editor};
use std::cell::RefCell;
use std::fmt::{self, Debug};
use std::rc::Rc;
use syn::{parse_str, BinOp, Expr, ExprLit, Lit, Local, Pat, Stmt};
use utilities::variant_name;

mod helper;
mod storage;
mod value;

use helper::NadirHelper;
use storage::{Storage, StorageErrors};
use value::{Value, ValueErrors};

#[derive(Debug)]
pub enum NadirErrors {
    InvalidOperation,
    NameNotString(String),
    StorageErrors(StorageErrors),
    UnsupportedExpr(String),
    UnsupportedLiteral(String),
    UnsupportedLocalNamePattern(String),
    UnsupportedPathIdent,
    UnsupportedPattern(String),
    UnsupportedStatement(String),
    ValueErrors(ValueErrors),
}

impl From<StorageErrors> for NadirErrors {
    fn from(value: StorageErrors) -> Self {
        Self::StorageErrors(value)
    }
}

impl From<ValueErrors> for NadirErrors {
    fn from(value: ValueErrors) -> Self {
        Self::ValueErrors(value)
    }
}

impl fmt::Display for NadirErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let error_message = match self {
            NadirErrors::UnsupportedExpr(s) => format!("unsupported expression '{}'", s),
            NadirErrors::InvalidOperation => format!("invalid operation"),
            NadirErrors::NameNotString(s) => format!("name {} was not a String", s),
            NadirErrors::StorageErrors(err) => format!("{}", err),
            NadirErrors::UnsupportedLiteral(s) => format!("unsupported literal '{}'", s),
            NadirErrors::UnsupportedLocalNamePattern(s) => {
                format!("unsupported variable name pattern '{}'", s)
            }
            NadirErrors::UnsupportedPathIdent => {
                format!("variable identifier expected to be a single string")
            }
            NadirErrors::UnsupportedPattern(s) => format!("unsupported pattern '{}'", s),
            NadirErrors::UnsupportedStatement(s) => format!("unsupported statement '{}'", s),
            NadirErrors::ValueErrors(err) => format!("{}", err),
        };

        // Wrap the error message in red
        let error_message = Colour::Red.paint(error_message);
        write!(f, "{}", error_message)
    }
}

fn main() {
    // This store holds variables as Box<dyn Any> so we can put any type into it.
    let storage = Rc::new(RefCell::new(Storage::default()));

    let config = Config::builder()
        .history_ignore_space(true)
        .completion_type(CompletionType::List)
        .edit_mode(EditMode::Emacs)
        .build();
    let h = NadirHelper::new(storage.clone());

    // `()` can be used when no completer is required
    let mut rl = Editor::with_config(config).expect("Failed to create rustyline editor");
    rl.set_helper(Some(h));
    // Determine the configuration directory
    let history_path = if let Some(mut history_path) = dirs::config_dir() {
        history_path.push("nadir");
        if !history_path.exists() {
            std::fs::create_dir_all(&history_path).expect("Could not create nadir directory");
        }
        history_path.push("cli_history.txt");

        if history_path.exists() {
            #[cfg(feature = "with-file-history")]
            rl.load_history(&history_path)
                .expect("Could not load history file.");
        }
        Some(history_path)
    } else {
        None
    };

    let prompt = "nadir> ";

    loop {
        // Display the prompt and read user input
        rl.helper_mut().expect("No helper").colored_prompt =
            format!("\x1b[38;2;246;189;96m{prompt}\x1b[0m");
        match rl.readline(&prompt) {
            Ok(line) => {
                // Add the input to history
                rl.add_history_entry(line.as_str())
                    .expect("Failed to add history entry");

                // Handle specific commands or process input
                match line.trim() {
                    "exit" => break,
                    "vars" => {
                        let storage = storage.borrow();
                        let max_key_length =
                            storage.0.keys().map(|key| key.len()).max().unwrap_or(0);

                        for (key, value) in &storage.0 {
                            let mut name = key.clone();
                            while name.len() < max_key_length {
                                name.push(' ');
                            }
                            println!("{} {}", name, value::label(&value.as_str()));
                        }
                    }
                    _ => {
                        let input = line.as_str();
                        match parse_str::<Stmt>(input) {
                            Ok(stmt) => match evaluate_statement(&stmt, &storage) {
                                Ok(_) => {}
                                Err(e) => eprintln!("{}", e),
                            },
                            Err(_) => match parse_str::<Expr>(input) {
                                Ok(expr) => match evaluate_expression(&expr, &storage) {
                                    Ok(v) => {
                                        if let Some(v) = v {
                                            println!("{:?}", v);
                                        } else {
                                            println!("None")
                                        }
                                    }
                                    Err(e) => eprintln!("{}", e),
                                },
                                Err(e) => eprintln!("Parsing Error: {}", e),
                            },
                        }
                    }
                }
            }
            Err(ReadlineError::Interrupted) => {
                break;
            }
            Err(ReadlineError::Eof) => {
                // Handle Ctrl+D (end-of-file) signal
                break;
            }
            Err(err) => {
                // Handle other potential errors
                println!("Error: {:?}", err);
                break;
            }
        }
    }
    if let Some(history_path) = &history_path {
        #[cfg(feature = "with-file-history")]
        rl.save_history(history_path)
            .expect("Failed to save history");
    }
}

fn evaluate_statement(stmt: &Stmt, storage: &Rc<RefCell<Storage>>) -> Result<(), NadirErrors> {
    match stmt {
        Stmt::Expr(expr, _) => {
            evaluate_expression(&expr, &storage)?;
        }
        Stmt::Local(local) => evaluate_local(local, &storage)?,
        Stmt::Item(_) => return Err(NadirErrors::UnsupportedStatement("Stmt::Item".to_string())),
        Stmt::Macro(_) => return Err(NadirErrors::UnsupportedStatement("Stmt::Macro".to_string())),
    }
    Ok(())
}

fn evaluate_local(local: &Local, storage: &Rc<RefCell<Storage>>) -> Result<(), NadirErrors> {
    let name = match &local.pat {
        Pat::Ident(ident) => Some(ident.ident.to_string()),
        Pat::Const(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Const".to_string(),
            ))
        }
        Pat::Lit(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Lit".to_string(),
            ))
        }
        Pat::Macro(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Macro".to_string(),
            ))
        }
        Pat::Or(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Or".to_string(),
            ))
        }
        Pat::Paren(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Paren".to_string(),
            ))
        }
        Pat::Path(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Path".to_string(),
            ))
        }
        Pat::Range(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Range".to_string(),
            ))
        }
        Pat::Reference(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Reference".to_string(),
            ))
        }
        Pat::Rest(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Rest".to_string(),
            ))
        }
        Pat::Slice(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Slice".to_string(),
            ))
        }
        Pat::Struct(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Struct".to_string(),
            ))
        }
        Pat::Tuple(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Tuple".to_string(),
            ))
        }
        Pat::TupleStruct(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::TupleStruct".to_string(),
            ))
        }
        Pat::Type(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Type".to_string(),
            ))
        }
        Pat::Verbatim(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Verbatim".to_string(),
            ))
        }
        Pat::Wild(_) => {
            return Err(NadirErrors::UnsupportedLocalNamePattern(
                "Pat::Wild".to_string(),
            ))
        }
        _ => return Err(NadirErrors::UnsupportedLocalNamePattern("".to_string())),
    };

    // we had an identifier, check if we have value, if we do, pattern is let name = value;
    // put the name value pair in storage
    if let Some(name) = name {
        if let Some(init) = &local.init {
            if let Some(value) = evaluate_expression(&init.expr, &storage)? {
                let mut storage = storage.borrow_mut();
                storage.insert(name, value)?;
            }
        }
    }
    Ok(())
}

fn evaluate_expression(
    expr: &Expr,
    storage: &Rc<RefCell<Storage>>,
) -> Result<Option<Value>, NadirErrors> {
    match expr {
        Expr::Assign(e) => {
            let name = evaluate_expression(&e.left, storage)?;
            let value = evaluate_expression(&e.right, storage)?;
            if let (Some(name), Some(value)) = (&name, &value) {
                let name = match name {
                    Value::String(name) => *name.clone(),
                    _ => return Err(NadirErrors::NameNotString(name.as_str())),
                };
                let mut storage = storage.borrow_mut();
                storage.insert(name, value.clone())?;
                Ok(Some(value.clone()))
            } else {
                unreachable!("all return types should be values")
            }
        }
        Expr::Binary(e) => {
            let left = evaluate_expression(&e.left, storage)?;
            let right = evaluate_expression(&e.right, storage)?;
            let value = match (left, right) {
                (Some(left), Some(right)) => match e.op {
                    BinOp::Add(_) => left.try_add(&right)?,
                    BinOp::Sub(_) => left.try_sub(&right)?,
                    BinOp::Mul(_) => left.try_mul(&right)?,
                    BinOp::Div(_) => left.try_div(&right)?,
                    _ => return Err(NadirErrors::InvalidOperation),
                },
                _ => todo!("add single ops like +="),
            };
            Ok(Some(value))
        }
        Expr::Lit(ExprLit { lit, .. }) => {
            let value = evaluate_literal(lit)?;
            Ok(Some(value))
        }
        Expr::Path(path) => {
            // i believe the only use case we have for path in this cli is simply
            // calling a variable from storage, the rest should be covered other under variants
            if let Some(name) = path.path.get_ident() {
                let storage = storage.borrow();
                let value = storage.get(&name.to_string())?;
                return Ok(Some(value.clone()));
            } else {
                return Err(NadirErrors::UnsupportedPathIdent);
            }
        }
        _ => Err(NadirErrors::UnsupportedExpr(variant_name(expr).to_string())),
    }
}

fn evaluate_literal(lit: &Lit) -> Result<Value, NadirErrors> {
    match lit {
        Lit::Int(lit_int) => Ok(Value::i64(
            lit_int
                .base10_digits()
                .parse::<i64>()
                .expect("could not parse to i64"),
        )),
        Lit::Float(lit_float) => Ok(Value::f64(
            lit_float
                .base10_digits()
                .parse::<f64>()
                .expect("could not parse to f64"),
        )),
        Lit::Str(lit_str) => Ok(Value::String(Box::new(lit_str.value().to_string()))),
        //Lit::Bool(lit_bool) => lit_bool.value.to_string(),
        _ => Err(NadirErrors::UnsupportedLiteral(lit.suffix().to_string())),
    }
}
// fn parse_expr(
//     pair: Pair<Rule>,
//     pratt: &PrattParser<Rule>,
//     storage: &mut Rc<RefCell<Storage>>,
// ) -> Result<Option<Value>, NadirErrors> {
//     dbg!(&pair);
//     match pair.as_rule() {
//         Rule::term => {
//             let inner_pair = pair.into_inner().next().unwrap();
//             parse_expr(inner_pair, pratt, storage)
//         }

//         Rule::vector => {
//             let inner_pairs = pair.into_inner();
//             let mut values = Vec::new();
//             for value_pair in inner_pairs {
//                 if let Some(value) = parse_expr(value_pair, pratt, storage)? {
//                     values.push(value.as_f64()?);
//                 } else {
//                     return Err(NadirErrors::CantParseVector);
//                 }
//             }
//             Ok(Some(Value::DVector(Box::new(DVector::from_vec(values)))))
//         }

//         // Handle matrix directly (non-arithmetic)
//         Rule::matrix => {
//             let inner_pairs = pair.into_inner();
//             let mut rows = Vec::new();
//             for row_pair in inner_pairs {
//                 let mut row = Vec::new();
//                 for value_pair in row_pair.into_inner() {
//                     if let Some(value) = parse_expr(value_pair, pratt, storage)? {
//                         row.push(value.as_f64()?);
//                     } else {
//                         return Err(NadirErrors::CantParseMatrix);
//                     }
//                 }
//                 rows.push(row);
//             }
//             let ncols = rows.get(0).map(|row| row.len()).unwrap_or(0);
//             let matrix_data: Vec<f64> = rows.concat();
//             Ok(Some(Value::DMatrix(Box::new(DMatrix::from_vec(
//                 rows.len(),
//                 ncols,
//                 matrix_data,
//             )))))
//         }

//         // Handle assignment: extract the variable name and expression.
//         Rule::assignment => {
//             let mut inner = pair.into_inner();
//             let name = inner.next().unwrap().as_str();
//             let expr = inner.next().unwrap();
//             let value = parse_expr(expr, pratt, storage)?;
//             if let Some(value) = &value {
//                 storage
//                     .borrow_mut()
//                     .insert(name.to_string(), value.clone())?;
//             }
//             Ok(value)
//         }

//         Rule::print_line => {
//             let next_pair = pair.into_inner().next().unwrap();
//             let value = parse_expr(next_pair, pratt, storage)?;
//             if let Some(ref value) = value {
//                 println!("{:?}", value);
//             }
//             Ok(value)
//         }

//         Rule::number => evaluate_number(pair),
//         Rule::identifier => evaluate_identifier(pair, storage),
//         Rule::function_call => evaluate_function_call(pair, pratt, storage, None),
//         Rule::struct_call => evaluate_struct_call(pair, pratt, storage),
//         Rule::instance_field => evaluate_instance_field(pair, storage),

//         // For everything else, assume it’s part of an arithmetic expression.
//         // Delegate these to the Pratt parser.
//         _ => {
//             return pratt
//                 .map_primary(|primary| match primary.as_rule() {
//                     Rule::number => evaluate_number(primary),
//                     Rule::identifier => evaluate_identifier(primary, storage),
//                     Rule::function_call => evaluate_function_call(primary, pratt, storage, None),
//                     Rule::struct_call => evaluate_struct_call(primary, pratt, storage),
//                     Rule::instance_field => evaluate_instance_field(primary, storage),
//                     _ => {
//                         dbg!(&primary.as_rule());
//                         Err(NadirErrors::UnexpectedRule(
//                             primary.as_str().to_string(),
//                         ))
//                     }
//                 })
//                 .map_prefix(|op, rhs| {
//                     rhs.map_or(Err(NadirErrors::EmptyValue), |rhs_value| {
//                         if let Some(rhs_value) = rhs_value {
//                             match op.as_rule() {
//                                 Rule::neg => Ok(Some(rhs_value.try_negative()?)),
//                                 _ => Err(NadirErrors::UnexpectedOperator(
//                                     op.as_str().to_string(),
//                                 )),
//                             }
//                         } else {
//                             Err(NadirErrors::EmptyValue)
//                         }
//                     })
//                 })
//                 .map_postfix(|lhs, op| {
//                     lhs.map_or(Err(NadirErrors::EmptyValue), |lhs_value| {
//                         if let Some(lhs_value) = lhs_value {
//                             match op.as_rule() {
//                                 Rule::fac => Ok(Some(lhs_value.try_factorial()?)),
//                                 _ => Err(NadirErrors::UnexpectedOperator(
//                                     op.as_str().to_string(),
//                                 )),
//                             }
//                         } else {
//                             Err(NadirErrors::EmptyValue)
//                         }
//                     })
//                 })
//                 .map_infix(|lhs, op, rhs| {
//                     lhs.and_then(|lhs_value| {
//                         if let Some(lhs_value) = lhs_value {
//                             rhs.map_or(Err(NadirErrors::EmptyValue), |rhs_value| {
//                                 if let Some(rhs_value) = rhs_value {
//                                     match op.as_rule() {
//                                         Rule::add => Ok(Some(lhs_value.try_add(&rhs_value)?)),
//                                         Rule::sub => Ok(Some(lhs_value.try_sub(&rhs_value)?)),
//                                         Rule::mul => Ok(Some(lhs_value.try_mul(&rhs_value)?)),
//                                         Rule::div => Ok(Some(lhs_value.try_div(&rhs_value)?)),
//                                         Rule::pow => Ok(Some(lhs_value.try_pow(&rhs_value)?)),
//                                         _ => Err(NadirErrors::UnexpectedOperator(
//                                             op.as_str().to_string(),
//                                         )),
//                                     }
//                                 } else {
//                                     Err(NadirErrors::EmptyValue)
//                                 }
//                             })
//                         } else {
//                             Err(NadirErrors::EmptyValue)
//                         }
//                     })
//                 })
//                 .parse(pair.into_inner());
//         }
//     }
// }

// fn evaluate_number(primary: Pair<Rule>) -> Result<Option<Value>, NadirErrors> {
//     let inner_pair = primary
//         .into_inner()
//         .next()
//         .ok_or_else(|| NadirErrors::InvalidNumber("Empty number".to_string()))?;

//     let number_str = inner_pair.as_str().to_string();

//     match inner_pair.as_rule() {
//         Rule::float => inner_pair
//             .as_str()
//             .parse::<f64>()
//             .map(|f| Some(Value::f64(f)))
//             .map_err(|_| NadirErrors::InvalidNumber(number_str)),
//         Rule::integer => inner_pair
//             .as_str()
//             .parse::<i64>()
//             .map(|i| Some(Value::i64(i)))
//             .map_err(|_| NadirErrors::InvalidNumber(number_str.clone())),
//         _ => Err(NadirErrors::InvalidNumber(number_str)),
//     }
// }

// fn evaluate_identifier(
//     primary: Pair<Rule>,
//     storage: &Rc<RefCell<Storage>>,
// ) -> Result<Option<Value>, NadirErrors> {
//     let storage = storage.borrow();
//     Ok(Some(storage.get(primary.as_str())?))
// }

// fn evaluate_function_call(
//     pair: Pair<Rule>,
//     pratt: &PrattParser<Rule>,
//     storage: &mut Rc<RefCell<Storage>>,
//     struct_name: Option<&str>,
// ) -> Result<Option<Value>, NadirErrors> {
//     let mut pairs = pair.into_inner();

//     let fn_name_pair = pairs.next().unwrap();
//     let fn_name = fn_name_pair.as_str();

//     let args_pair = pairs.next();

//     // Process each argument
//     let args = if let Some(args_pair) = args_pair {
//         let inner = args_pair.into_inner();
//         let mut args = Vec::new();
//         for arg in inner {
//             let value =
//                 parse_expr(arg, pratt, storage)?.ok_or_else(|| NadirErrors::EmptyValue)?;
//             args.push(value);
//         }
//         args
//     } else {
//         Vec::new()
//     };

//     // Match the function name and evaluate
//     if let Some(struct_name) = struct_name {
//         match (struct_name, fn_name) {
//             ("Quaternion", "new") => {
//                 if args.len() != 4 {
//                     return Err(NadirErrors::NumberOfArgs(
//                         fn_name.to_string(),
//                         "4".to_string(),
//                         args.len().to_string(),
//                     ));
//                 }
//                 let mut quat_args = [0.0; 4];
//                 for (i, arg) in args.iter().enumerate() {
//                     quat_args[i] = arg.as_f64()?;
//                 }
//                 Ok(Some(Value::Quaternion(Box::new(Quaternion::new(
//                     quat_args[0],
//                     quat_args[1],
//                     quat_args[2],
//                     quat_args[3],
//                 )))))
//             }
//             ("Quaternion", "rand") => {
//                 if args.len() != 0 {
//                     return Err(NadirErrors::NumberOfArgs(
//                         fn_name.to_string(),
//                         "0".to_string(),
//                         args.len().to_string(),
//                     ));
//                 }

//                 Ok(Some(Value::Quaternion(Box::new(Quaternion::rand()))))
//             }
//             ("UnitQuaternion", "new") => {
//                 if args.len() != 4 {
//                     return Err(NadirErrors::NumberOfArgs(
//                         fn_name.to_string(),
//                         "4".to_string(),
//                         args.len().to_string(),
//                     ));
//                 }
//                 let mut quat_args = [0.0; 4];
//                 for (i, arg) in args.iter().enumerate() {
//                     quat_args[i] = arg.as_f64()?;
//                 }
//                 Ok(Some(Value::UnitQuaternion(Box::new(UnitQuaternion::new(
//                     quat_args[0],
//                     quat_args[1],
//                     quat_args[2],
//                     quat_args[3],
//                 )))))
//             }
//             ("UnitQuaternion", "rand") => {
//                 if args.len() != 0 {
//                     return Err(NadirErrors::NumberOfArgs(
//                         fn_name.to_string(),
//                         "0".to_string(),
//                         args.len().to_string(),
//                     ));
//                 }

//                 Ok(Some(Value::UnitQuaternion(
//                     Box::new(UnitQuaternion::rand()),
//                 )))
//             }
//             _ => Err(NadirErrors::FunctionNotFound(format!(
//                 "{}::{}",
//                 struct_name, fn_name
//             ))),
//         }
//     } else {
//         match fn_name {
//             "quat" => {
//                 if args.len() != 4 {
//                     return Err(NadirErrors::NumberOfArgs(
//                         fn_name.to_string(),
//                         "4".to_string(),
//                         args.len().to_string(),
//                     ));
//                 }
//                 let mut quat_args = [0.0; 4];
//                 for (i, arg) in args.iter().enumerate() {
//                     quat_args[i] = arg.as_f64()?;
//                 }
//                 Ok(Some(Value::Quaternion(Box::new(Quaternion::new(
//                     quat_args[0],
//                     quat_args[1],
//                     quat_args[2],
//                     quat_args[3],
//                 )))))
//             }
//             _ => Err(NadirErrors::FunctionNotFound(fn_name.to_string())),
//         }
//     }
// }
// fn evaluate_struct_call(
//     pair: Pair<Rule>,
//     pratt: &PrattParser<Rule>,
//     storage: &mut Rc<RefCell<Storage>>,
// ) -> Result<Option<Value>, NadirErrors> {
//     // Extract the function_call pair
//     let mut pairs = pair.into_inner();
//     let struct_name_pair = pairs.next().unwrap();
//     let struct_name = struct_name_pair.as_str();
//     let fn_call_pair = pairs.next().unwrap();
//     evaluate_function_call(fn_call_pair, pratt, storage, Some(struct_name))
// }

// fn evaluate_instance_field(
//     pair: Pair<Rule>,
//     storage: &Rc<RefCell<Storage>>,
// ) -> Result<Option<Value>, NadirErrors> {
//     let storage = storage.borrow();
//     let mut pairs = pair.into_inner();
//     let instance_name = pairs.next().unwrap().as_str();
//     let instance = storage.get(instance_name)?;
//     let field = pairs.next().unwrap().as_str();
//     match instance {
//         Value::Quaternion(q) => match field {
//             "x" => Ok(Some(Value::f64(q.x))),
//             "y" => Ok(Some(Value::f64(q.y))),
//             "z" => Ok(Some(Value::f64(q.z))),
//             "w" => Ok(Some(Value::f64(q.w))),
//             _ => Err(NadirErrors::InvalidField(
//                 instance_name.to_string(),
//                 field.to_string(),
//             )),
//         },
//         Value::UnitQuaternion(q) => match field {
//             "x" => Ok(Some(Value::f64(q.0.x))),
//             "y" => Ok(Some(Value::f64(q.0.y))),
//             "z" => Ok(Some(Value::f64(q.0.z))),
//             "w" => Ok(Some(Value::f64(q.0.w))),
//             _ => Err(NadirErrors::InvalidField(
//                 instance_name.to_string(),
//                 field.to_string(),
//             )),
//         },
//         _ => Err(NadirErrors::InvalidField(
//             instance_name.to_string(),
//             field.to_string(),
//         )),
//     }
// }
