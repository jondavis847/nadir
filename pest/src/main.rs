use ansi_term::Colour;
use pest::iterators::{Pair, Pairs};
use pest::pratt_parser::{Assoc, Op, PrattParser};
use pest::Parser;
use pest_derive::Parser;
use rotations::prelude::{Quaternion, UnitQuaternion};
use rustyline::error::ReadlineError;
use rustyline::{CompletionType, Config, EditMode, Editor};
use std::cell::RefCell;
use std::fmt::{self, Debug};
use std::rc::Rc;

mod helper;
mod storage;
mod value;

use helper::NadirHelper;
use storage::{Storage, StorageErrors};
use value::{Value, ValueErrors};

#[derive(Parser)]
#[grammar = "main.pest"] // relative path to your .pest file
struct NadirParser;

#[derive(Debug)]
pub enum NadirParserErrors {
    ArgumentValueIsNone(String),
    EmptyPairs(String),
    ExpectedFunctionArguments(String),
    ExpectedFunctionCall(String),
    ExpectedIdentifier(String),
    FunctionNotFound(String),
    InvalidNumber(String),
    MissingFunctionArguments,
    MissingFunctionName,
    NumberOfArgs(String, String, String),
    StorageErrors(StorageErrors),
    UnexpectedOperator(String),
    ValueErrors(ValueErrors),
}

impl From<StorageErrors> for NadirParserErrors {
    fn from(value: StorageErrors) -> Self {
        Self::StorageErrors(value)
    }
}

impl From<ValueErrors> for NadirParserErrors {
    fn from(value: ValueErrors) -> Self {
        Self::ValueErrors(value)
    }
}

impl fmt::Display for NadirParserErrors {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let error_message = match self {
            NadirParserErrors::ArgumentValueIsNone(arg) => {
                format!("argument value was None: {}", arg)
            }
            NadirParserErrors::EmptyPairs(arg) => format!("pest parsing pairs were empty: {}", arg),
            NadirParserErrors::ExpectedFunctionArguments(arg) => {
                format!("expected rule was function_arguments but got: {}", arg)
            }
            NadirParserErrors::ExpectedFunctionCall(arg) => {
                format!("expected rule was function_call but got: {}", arg)
            }
            NadirParserErrors::ExpectedIdentifier(arg) => {
                format!("expected rule was identifier but got: {}", arg)
            }
            NadirParserErrors::FunctionNotFound(func) => format!("function not found: {}", func),
            NadirParserErrors::InvalidNumber(num) => format!("invalid number: {}", num),
            NadirParserErrors::MissingFunctionArguments => {
                "was not able to parse function arguments".to_string()
            }
            NadirParserErrors::MissingFunctionName => {
                "was not able to parse a function name".to_string()
            }
            NadirParserErrors::NumberOfArgs(func, expected, got) => {
                format!(
                    "incorrect number of args to function '{}'. expected {}, got {}",
                    func, expected, got
                )
            }
            NadirParserErrors::StorageErrors(err) => format!("{}", err),
            NadirParserErrors::UnexpectedOperator(op) => format!("unexpected operator: {}", op),
            NadirParserErrors::ValueErrors(err) => format!("{}", err),
        };

        // Wrap the error message in red
        let error_message = Colour::Red.paint(error_message);
        write!(f, "{}", error_message)
    }
}

fn main() {
    // This store holds variables as Box<dyn Any> so we can put any type into it.
    let mut storage = Rc::new(RefCell::new(Storage::default()));

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

    let pratt = PrattParser::new()
        .op(Op::infix(Rule::add, Assoc::Left) | Op::infix(Rule::sub, Assoc::Left))
        .op(Op::infix(Rule::mul, Assoc::Left) | Op::infix(Rule::div, Assoc::Left))
        .op(Op::infix(Rule::pow, Assoc::Right))
        .op(Op::prefix(Rule::neg))
        .op(Op::postfix(Rule::fac));

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
                        let keys: Vec<&String> = storage.get_names();
                        for name in keys {
                            println!("{}", name);
                        }
                    }
                    _ => {
                        // Parse the input using the "line" rule.
                        let parse_result = NadirParser::parse(Rule::line, &line);
                        match parse_result {
                            Ok(mut pairs) => {
                                match parse_expr(&mut pairs, &pratt, &mut storage) {
                                    Ok(_) => {}
                                    Err(e) => eprintln!("{e}"),
                                };
                            }
                            Err(e) => {
                                eprintln!("Parsing error: {}", e);
                            }
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

fn parse_expr(
    pairs: &mut Pairs<Rule>,
    pratt: &PrattParser<Rule>,
    storage: &mut Rc<RefCell<Storage>>,
) -> Result<Option<Value>, NadirParserErrors> {
    pratt
        .map_primary(|primary| {
            match primary.as_rule() {
                Rule::number => evaluate_number(primary),
                Rule::identifier => evaluate_identifier(primary, storage),
                Rule::function_call => evaluate_function_call(primary, pratt, storage, None),
                Rule::struct_call => evaluate_struct_call(primary, pratt, storage),
                Rule::print_line => {
                    let value = parse_expr(&mut primary.into_inner(), pratt, storage)?;
                    if let Some(value) = &value {
                        println!("{:?}", value);
                    };
                    Ok(value)
                }
                Rule::assignment => {
                    let mut pairs = primary.into_inner();
                    let name = pairs.next().unwrap().as_str();
                    let expr = pairs.next().unwrap();
                    let value = parse_expr(&mut expr.into_inner(), pratt, storage)?;
                    if let Some(value) = &value {
                        let mut storage = storage.borrow_mut();
                        storage.insert(name.to_string(), value.clone())?;
                    };
                    Ok(value)
                }
                _ => parse_expr(&mut primary.into_inner(), pratt, storage), // For any other rule, return None
            }
        })
        .map_prefix(|op, rhs| {
            rhs.and_then(|rhs_value| {
                rhs_value
                    .map(|value| match op.as_rule() {
                        Rule::neg => Ok(value.try_negative()?),
                        _ => Err(NadirParserErrors::UnexpectedOperator(
                            op.as_str().to_string(),
                        )),
                    })
                    .transpose()
            })
        })
        .map_postfix(|lhs, op| {
            lhs.and_then(|lhs_value| {
                lhs_value
                    .map(|value| match op.as_rule() {
                        Rule::fac => Ok(value.try_factorial()?),
                        _ => Err(NadirParserErrors::UnexpectedOperator(
                            op.as_str().to_string(),
                        )),
                    })
                    .transpose()
            })
        })
        .map_infix(|lhs, op, rhs| {
            lhs.and_then(|lhs_value| {
                rhs.and_then(|rhs_value| {
                    lhs_value
                        .and_then(|lhs| {
                            rhs_value.map(|rhs| match op.as_rule() {
                                Rule::add => Ok(lhs.try_add(&rhs)?),
                                Rule::sub => Ok(lhs.try_sub(&rhs)?),
                                Rule::mul => Ok(lhs.try_mul(&rhs)?),
                                Rule::div => Ok(lhs.try_div(&rhs)?),
                                Rule::pow => Ok(lhs.try_pow(&rhs)?),
                                _ => Err(NadirParserErrors::UnexpectedOperator(
                                    op.as_str().to_string(),
                                )),
                            })
                        })
                        .transpose()
                })
            })
        })
        .parse(pairs)
}

fn evaluate_number(primary: Pair<Rule>) -> Result<Option<Value>, NadirParserErrors> {
    let inner_pair = primary
        .into_inner()
        .next()
        .ok_or_else(|| NadirParserErrors::InvalidNumber("Empty number".to_string()))?;

    let number_str = inner_pair.as_str().to_string();

    match inner_pair.as_rule() {
        Rule::float => inner_pair
            .as_str()
            .parse::<f64>()
            .map(|f| Some(Value::f64(f)))
            .map_err(|_| NadirParserErrors::InvalidNumber(number_str)),
        Rule::integer => inner_pair
            .as_str()
            .parse::<i64>()
            .map(|i| Some(Value::i64(i)))
            .map_err(|_| NadirParserErrors::InvalidNumber(number_str.clone())),
        _ => Err(NadirParserErrors::InvalidNumber(number_str)),
    }
}

fn evaluate_identifier(
    primary: Pair<Rule>,
    storage: &Rc<RefCell<Storage>>,
) -> Result<Option<Value>, NadirParserErrors> {
    let storage = storage.borrow();
    Ok(Some(storage.get(primary.as_str())?))
}

fn evaluate_function_call(
    pair: Pair<Rule>,
    pratt: &PrattParser<Rule>,
    storage: &mut Rc<RefCell<Storage>>,
    struct_name: Option<&str>,
) -> Result<Option<Value>, NadirParserErrors> {
    let mut pairs = pair.into_inner();
    let fn_name_pair = pairs.next().unwrap();
    let fn_name = fn_name_pair.as_str();

    let args_pair = pairs.next();

    // Process each argument
    let args = if let Some(args_pair) = args_pair {
        let inner = args_pair.into_inner();
        let mut args = Vec::new();
        for arg in inner {
            let arg_clone = arg.clone();
            let value = parse_expr(&mut arg_clone.into_inner(), pratt, storage)?
                .ok_or_else(|| NadirParserErrors::ArgumentValueIsNone(arg.as_str().to_string()))?;
            args.push(value);
        }
        args
    } else {
        Vec::new()
    };

    // Match the function name and evaluate
    if let Some(struct_name) = struct_name {
        match (struct_name, fn_name) {
            ("Quaternion", "new") => {
                if args.len() != 4 {
                    return Err(NadirParserErrors::NumberOfArgs(
                        fn_name.to_string(),
                        "4".to_string(),
                        args.len().to_string(),
                    ));
                }
                let mut quat_args = [0.0; 4];
                for (i, arg) in args.iter().enumerate() {
                    quat_args[i] = arg.as_f64()?;
                }
                Ok(Some(Value::Quaternion(Box::new(Quaternion::new(
                    quat_args[0],
                    quat_args[1],
                    quat_args[2],
                    quat_args[3],
                )))))
            }
            ("Quaternion", "rand") => {
                if args.len() != 0 {
                    return Err(NadirParserErrors::NumberOfArgs(
                        fn_name.to_string(),
                        "0".to_string(),
                        args.len().to_string(),
                    ));
                }

                Ok(Some(Value::Quaternion(Box::new(Quaternion::rand()))))
            }
            ("UnitQuaternion", "new") => {
                if args.len() != 4 {
                    return Err(NadirParserErrors::NumberOfArgs(
                        fn_name.to_string(),
                        "4".to_string(),
                        args.len().to_string(),
                    ));
                }
                let mut quat_args = [0.0; 4];
                for (i, arg) in args.iter().enumerate() {
                    quat_args[i] = arg.as_f64()?;
                }
                Ok(Some(Value::UnitQuaternion(Box::new(UnitQuaternion::new(
                    quat_args[0],
                    quat_args[1],
                    quat_args[2],
                    quat_args[3],
                )))))
            }
            ("UnitQuaternion", "rand") => {
                if args.len() != 0 {
                    return Err(NadirParserErrors::NumberOfArgs(
                        fn_name.to_string(),
                        "0".to_string(),
                        args.len().to_string(),
                    ));
                }

                Ok(Some(Value::UnitQuaternion(
                    Box::new(UnitQuaternion::rand()),
                )))
            }
            _ => Err(NadirParserErrors::FunctionNotFound(format!(
                "{}::{}",
                struct_name, fn_name
            ))),
        }
    } else {
        match fn_name {
            "quat" => {
                if args.len() != 4 {
                    return Err(NadirParserErrors::NumberOfArgs(
                        fn_name.to_string(),
                        "4".to_string(),
                        args.len().to_string(),
                    ));
                }
                let mut quat_args = [0.0; 4];
                for (i, arg) in args.iter().enumerate() {
                    quat_args[i] = arg.as_f64()?;
                }
                Ok(Some(Value::Quaternion(Box::new(Quaternion::new(
                    quat_args[0],
                    quat_args[1],
                    quat_args[2],
                    quat_args[3],
                )))))
            }
            _ => Err(NadirParserErrors::FunctionNotFound(fn_name.to_string())),
        }
    }
}
fn evaluate_struct_call(
    pair: Pair<Rule>,
    pratt: &PrattParser<Rule>,
    storage: &mut Rc<RefCell<Storage>>,
) -> Result<Option<Value>, NadirParserErrors> {
    // Extract the function_call pair
    let mut pairs = pair.into_inner();
    let struct_name_pair = pairs.next().unwrap();
    let struct_name = struct_name_pair.as_str();
    let fn_call_pair = pairs.next().unwrap();
    evaluate_function_call(fn_call_pair, pratt, storage, Some(struct_name))
}
