use ansi_term::Colour;
use nalgebra::{DMatrix, DVector, Dim, Dyn, RowDVector, RowVector, VecStorage};
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
    CantParseVector,
    CantParseMatrix,
    EmptyPairs(String),
    EmptyValue,
    ExpectedFunctionArguments(String),
    ExpectedFunctionCall(String),
    ExpectedIdentifier(String),
    FunctionNotFound(String),
    InvalidField(String, String),
    InvalidNumber(String),
    MatrixRowLengthMismatch,
    MissingFunctionArguments,
    MissingFunctionName,
    NumberOfArgs(String, String, String),
    StorageErrors(StorageErrors),
    UnexpectedOperator(String),
    UnexpectedRule(String),
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
            NadirParserErrors::CantParseVector => "could not parse vector".to_string(),
            NadirParserErrors::CantParseMatrix => "could not parse matrix".to_string(),
            NadirParserErrors::EmptyValue => format!("pest parsing value was empty"),
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
            NadirParserErrors::InvalidField(instance, field) => {
                format!("invalid field {field} for variable {instance}")
            }
            NadirParserErrors::InvalidNumber(num) => format!("invalid number: {}", num),
            NadirParserErrors::MatrixRowLengthMismatch => {
                format!("matrix cannot have rows of different lengths")
            }
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
            NadirParserErrors::UnexpectedRule(rule) => format!("unexpected rule: {}", rule),
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
                                if let Some(line_pair) = pairs.next() {
                                    // get to next level, with is a silent_line or print_line
                                    let print_or_silent = line_pair.into_inner().next().unwrap();
                                    match parse_expr(print_or_silent, &pratt, &mut storage) {
                                        Ok(_) => {}
                                        Err(e) => eprintln!("{e}"),
                                    };
                                }
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
    pair: Pair<Rule>,
    pratt: &PrattParser<Rule>,
    storage: &mut Rc<RefCell<Storage>>,
) -> Result<Option<Value>, NadirParserErrors> {
    dbg!(&pair);
    match pair.as_rule() {
        Rule::expr => {
            let inner_pair = pair.into_inner().next().unwrap();
            parse_expr(inner_pair, pratt, storage)
        }
        Rule::term => {
            let inner_pair = pair.into_inner().next().unwrap();
            parse_expr(inner_pair, pratt, storage)
        }
        Rule::vector => {
            let elements_pair = pair.into_inner().next().unwrap();
            let comma_separated_pair = elements_pair.into_inner().next().unwrap();
            let value_pairs = comma_separated_pair.into_inner();

            let mut values = Vec::new();
            for value_pair in value_pairs {
                if let Some(value) = parse_expr(value_pair, pratt, storage)? {
                    values.push(value.as_f64()?);
                } else {
                    return Err(NadirParserErrors::CantParseVector);
                }
            }
            Ok(Some(Value::DVector(Box::new(DVector::from_vec(values)))))
        }

        Rule::matrix => {
            let row_pairs = pair.into_inner();
            let mut matrix = VecStorage::new(1, 1, Vec::new());
            for row_pair in row_pairs {
                let space_separated_pair = row_pair.into_inner().next().unwrap();
                for value_pair in space_separated_pair.into_inner() {
                    if let Some(value) = parse_expr(value_pair, pratt, storage)? {
                        row.push(value.as_f64()?);
                    } else {
                        return Err(NadirParserErrors::CantParseMatrix);
                    }
                }
                rows.push(row);
            }
            // check that all rows have same len
            let row_len = rows[0].len();
            if rows.len() > 1 {
                for i in 1..rows.len() {
                    if rows[i].len() != row_len {
                        return Err(NadirParserErrors::MatrixRowLengthMismatch);
                    }
                }
            }
            dbg!(&rows);
            let matrix = DMatrix::from_rows(&rows);
            dbg!(&matrix);
            Ok(Some(Value::DMatrix(Box::new(matrix))))
        }

        Rule::assignment => {
            let mut inner = pair.into_inner();
            let name = inner.next().unwrap().as_str();
            let expr = inner.next().unwrap();
            let value = parse_expr(expr, pratt, storage)?;
            if let Some(value) = &value {
                storage
                    .borrow_mut()
                    .insert(name.to_string(), value.clone())?;
            }
            Ok(value)
        }

        Rule::print_line => {
            let next_pair = pair.into_inner().next().unwrap();
            let value = parse_expr(next_pair, pratt, storage)?;
            if let Some(ref value) = value {
                println!("{:?}", value);
            }
            Ok(value)
        }
        Rule::silent_line => {
            let next_pair = pair.into_inner().next().unwrap();
            parse_expr(next_pair, pratt, storage)?;
            Ok(None)
        }

        Rule::number => evaluate_number(pair),
        Rule::identifier => evaluate_identifier(pair, storage),
        Rule::function_call => evaluate_function_call(pair, pratt, storage, None),
        Rule::struct_call => evaluate_struct_call(pair, pratt, storage),
        Rule::instance_field => evaluate_instance_field(pair, storage),
        Rule::elements => evaluate_elements(pair, pratt, storage),

        // For everything else, assume it’s part of an arithmetic expression.
        // Delegate these to the Pratt parser.
        Rule::math => {
            return pratt
                .map_primary(|primary| parse_expr(primary, pratt, storage))
                .map_prefix(|op, rhs| {
                    rhs.map_or(Err(NadirParserErrors::EmptyValue), |rhs_value| {
                        if let Some(rhs_value) = rhs_value {
                            match op.as_rule() {
                                Rule::neg => Ok(Some(rhs_value.try_negative()?)),
                                _ => Err(NadirParserErrors::UnexpectedOperator(
                                    op.as_str().to_string(),
                                )),
                            }
                        } else {
                            Err(NadirParserErrors::EmptyValue)
                        }
                    })
                })
                .map_postfix(|lhs, op| {
                    lhs.map_or(Err(NadirParserErrors::EmptyValue), |lhs_value| {
                        if let Some(lhs_value) = lhs_value {
                            match op.as_rule() {
                                Rule::fac => Ok(Some(lhs_value.try_factorial()?)),
                                _ => Err(NadirParserErrors::UnexpectedOperator(
                                    op.as_str().to_string(),
                                )),
                            }
                        } else {
                            Err(NadirParserErrors::EmptyValue)
                        }
                    })
                })
                .map_infix(|lhs, op, rhs| {
                    lhs.and_then(|lhs_value| {
                        if let Some(lhs_value) = lhs_value {
                            rhs.map_or(Err(NadirParserErrors::EmptyValue), |rhs_value| {
                                if let Some(rhs_value) = rhs_value {
                                    match op.as_rule() {
                                        Rule::add => Ok(Some(lhs_value.try_add(&rhs_value)?)),
                                        Rule::sub => Ok(Some(lhs_value.try_sub(&rhs_value)?)),
                                        Rule::mul => Ok(Some(lhs_value.try_mul(&rhs_value)?)),
                                        Rule::div => Ok(Some(lhs_value.try_div(&rhs_value)?)),
                                        Rule::pow => Ok(Some(lhs_value.try_pow(&rhs_value)?)),
                                        _ => Err(NadirParserErrors::UnexpectedOperator(
                                            op.as_str().to_string(),
                                        )),
                                    }
                                } else {
                                    Err(NadirParserErrors::EmptyValue)
                                }
                            })
                        } else {
                            Err(NadirParserErrors::EmptyValue)
                        }
                    })
                })
                .parse(pair.into_inner());
        }
        _ => Err(NadirParserErrors::UnexpectedRule(pair.as_str().to_string())),
    }
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
            let value =
                parse_expr(arg, pratt, storage)?.ok_or_else(|| NadirParserErrors::EmptyValue)?;
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

fn evaluate_instance_field(
    pair: Pair<Rule>,
    storage: &Rc<RefCell<Storage>>,
) -> Result<Option<Value>, NadirParserErrors> {
    let storage = storage.borrow();
    let mut pairs = pair.into_inner();
    let instance_name = pairs.next().unwrap().as_str();
    let instance = storage.get(instance_name)?;
    let field = pairs.next().unwrap().as_str();
    match instance {
        Value::Quaternion(q) => match field {
            "x" => Ok(Some(Value::f64(q.x))),
            "y" => Ok(Some(Value::f64(q.y))),
            "z" => Ok(Some(Value::f64(q.z))),
            "w" => Ok(Some(Value::f64(q.w))),
            _ => Err(NadirParserErrors::InvalidField(
                instance_name.to_string(),
                field.to_string(),
            )),
        },
        Value::UnitQuaternion(q) => match field {
            "x" => Ok(Some(Value::f64(q.0.x))),
            "y" => Ok(Some(Value::f64(q.0.y))),
            "z" => Ok(Some(Value::f64(q.0.z))),
            "w" => Ok(Some(Value::f64(q.0.w))),
            _ => Err(NadirParserErrors::InvalidField(
                instance_name.to_string(),
                field.to_string(),
            )),
        },
        _ => Err(NadirParserErrors::InvalidField(
            instance_name.to_string(),
            field.to_string(),
        )),
    }
}

fn evaluate_elements(
    pair: Pair<Rule>,
    pratt: &PrattParser<Rule>,
    storage: &mut Rc<RefCell<Storage>>,
) -> Result<Option<Value>, NadirParserErrors> {
    // this pair is Rule::element, next pair must be Rule::comma_separated
    // get the next rule
    let commma_separated = pair.into_inner().next().unwrap();
    let elements = commma_separated.into_inner();
    let mut vec = Vec::new();
    for element in elements {
        if let Some(value) = parse_expr(element, pratt, storage)? {
            vec.push(value.as_f64()?);
        } else {
            return Err(NadirParserErrors::EmptyValue);
        }
    }
    Ok(Some(Value::DVector(Box::new(DVector::from(vec)))))
}
