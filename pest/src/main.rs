use ansi_term::Colour;
use nalgebra::{DMatrix, DVector};
use pest::iterators::Pair;
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
use value::{IndexStyle, Range, Value, ValueErrors};

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
    OutOfBoundsIndex(String, String),
    StorageErrors(StorageErrors),
    UnexpectedOperator(String),
    UnexpectedRule(Rule),
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
            NadirParserErrors::OutOfBoundsIndex(max, ind) => {
                format!("max size was {max} but index was {ind}")
            }
            NadirParserErrors::StorageErrors(err) => format!("{}", err),
            NadirParserErrors::UnexpectedOperator(op) => format!("unexpected operator: {}", op),
            NadirParserErrors::UnexpectedRule(rule) => format!("unexpected rule: {:?}", rule),
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
                                    dbg!(&line_pair);
                                    // get to next level, with is a silent_line or print_line
                                    let print_or_silent = line_pair.into_inner().next().unwrap();
                                    match parse_expr(print_or_silent, &mut storage) {
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
    storage: &mut Rc<RefCell<Storage>>,
) -> Result<Option<Value>, NadirParserErrors> {
    match pair.as_rule() {
        Rule::additive => {
            let mut inner_pairs = pair.into_inner();
            let mut value = parse_expr(inner_pairs.next().unwrap(), storage)?.unwrap();

            while let (Some(op_pair), Some(right_pair)) = (inner_pairs.next(), inner_pairs.next()) {
                let right = parse_expr(right_pair, storage)?.unwrap();
                value = match op_pair.as_rule() {
                    Rule::add => value.try_add(&right)?,
                    Rule::sub => value.try_sub(&right)?,
                    _ => unreachable!(),
                };
            }

            Ok(Some(value))
        }
        Rule::assignment => {
            let mut inner = pair.into_inner();
            let name = inner.next().unwrap().as_str();
            let expr = inner.next().unwrap();
            let value = parse_expr(expr, storage)?;
            if let Some(value) = &value {
                storage
                    .borrow_mut()
                    .insert(name.to_string(), value.clone())?;
            }
            Ok(value)
        }
        Rule::expr => {
            let inner_pair = pair.into_inner().next().unwrap();
            parse_expr(inner_pair, storage)
        }
        Rule::exponential => {
            let mut inner_pairs = pair.into_inner();
            let mut value = parse_expr(inner_pairs.next().unwrap(), storage)?.unwrap();

            while let (Some(op_pair), Some(right_pair)) = (inner_pairs.next(), inner_pairs.next()) {
                let right = parse_expr(right_pair, storage)?.unwrap();
                value = match op_pair.as_rule() {
                    Rule::pow => value.try_pow(&right)?,
                    _ => unreachable!(),
                };
            }
            Ok(Some(value))
        }
        Rule::float => Ok(Some(Value::f64(pair.as_str().parse::<f64>().unwrap()))),
        Rule::function_call => evaluate_function_call(pair, storage, None),
        Rule::identifier => Ok(Some(storage.borrow().get(pair.as_str())?)),
        Rule::instance_field => evaluate_instance_field(pair, storage),
        Rule::instance_call => evaluate_instance_call(pair, storage),
        Rule::integer => Ok(Some(Value::i64(pair.as_str().parse::<i64>().unwrap()))),
        Rule::matrix => {
            let row_pairs = pair.into_inner();
            let mut rows: Vec<Vec<f64>> = Vec::new();
            // Parse each row from pest.
            for row_pair in row_pairs {
                let space_separated_pair = row_pair.into_inner().next().unwrap();
                let mut row: Vec<f64> = Vec::new();
                for value_pair in space_separated_pair.into_inner() {
                    if let Some(value) = parse_expr(value_pair, storage)? {
                        row.push(value.as_f64()?);
                    } else {
                        return Err(NadirParserErrors::CantParseMatrix);
                    }
                }
                rows.push(row);
            }

            // Check that we have at least one row and that all rows are of equal length.
            if rows.is_empty() {
                return Err(NadirParserErrors::CantParseMatrix);
            }
            let row_len = rows[0].len();
            if !rows.iter().all(|r| r.len() == row_len) {
                return Err(NadirParserErrors::MatrixRowLengthMismatch);
            }

            // Flatten the matrix in row-major order.
            let flat_data: Vec<f64> = rows.into_iter().flatten().collect();
            let num_rows = flat_data.len() / row_len;

            // Create the DMatrix using from_row_slice which accepts row-major data.
            let matrix = DMatrix::from_row_slice(num_rows, row_len, &flat_data);

            Ok(Some(Value::Matrix(Box::new(matrix))))
        }
        Rule::matrix_index => {
            let mut inner_pairs = pair.into_inner();
            let matrix_pair = inner_pairs.next().unwrap();
            let row_index_pair = inner_pairs.next().unwrap();
            let col_index_pair = inner_pairs.next().unwrap();

            // Determine if row/col is a full selector or specific index
            let row_selection = if row_index_pair.as_str() == ":" {
                None // None signifies full row selection
            } else {
                Some(parse_expr(row_index_pair, storage)?.unwrap().as_usize()?)
            };

            let col_selection = if col_index_pair.as_str() == ":" {
                None // None signifies full column selection
            } else {
                Some(parse_expr(col_index_pair, storage)?.unwrap().as_usize()?)
            };

            let matrix = match parse_expr(matrix_pair, storage)?.unwrap() {
                Value::Matrix(matrix) => *matrix,
                _ => unreachable!("shouldn't be here if it's not a matrix"),
            };

            // Extract the desired elements based on the selection
            match (row_selection, col_selection) {
                (Some(row), Some(col)) => {
                    // Specific element
                    if row > matrix.nrows() {
                        return Err(NadirParserErrors::OutOfBoundsIndex(
                            row.to_string(),
                            matrix.nrows().to_string(),
                        ));
                    }
                    if col > matrix.ncols() {
                        return Err(NadirParserErrors::OutOfBoundsIndex(
                            col.to_string(),
                            matrix.ncols().to_string(),
                        ));
                    }
                    let value = matrix.get((row, col)).unwrap();
                    Ok(Some(Value::f64(*value)))
                }
                (Some(row), None) => {
                    // Entire row
                    let row_vec = matrix.row(row).transpose().into_owned();
                    Ok(Some(Value::Vector(Box::new(row_vec))))
                }
                (None, Some(col)) => {
                    // Entire column
                    let col_vec = matrix.column(col).into_owned();
                    Ok(Some(Value::Vector(Box::new(col_vec))))
                }
                (None, None) => {
                    // Entire matrix
                    Ok(Some(Value::Matrix(Box::new(matrix.clone()))))
                }
            }
        }
        Rule::multiplicative => {
            let mut inner_pairs = pair.into_inner();
            let mut value = parse_expr(inner_pairs.next().unwrap(), storage)?.unwrap();

            while let (Some(op_pair), Some(right_pair)) = (inner_pairs.next(), inner_pairs.next()) {
                let right = parse_expr(right_pair, storage)?.unwrap();
                value = match op_pair.as_rule() {
                    Rule::mul => value.try_mul(&right)?,
                    Rule::div => value.try_div(&right)?,
                    _ => unreachable!(),
                };
            }

            Ok(Some(value))
        }
        Rule::print_line => {
            let next_pair = pair.into_inner().next().unwrap();
            let value = parse_expr(next_pair, storage)?;
            if let Some(ref value) = value {
                println!("{:?}", value);
            }
            Ok(value)
        }
        Rule::silent_line => {
            let next_pair = pair.into_inner().next().unwrap();
            parse_expr(next_pair, storage)?;
            Ok(None)
        }
        Rule::string => {
            let parsed_str = pair.as_str();
            let unquoted_str = &parsed_str[1..parsed_str.len() - 1]; // Remove the first and last character (the quotes)
            Ok(Some(Value::String(Box::new(unquoted_str.to_string()))))
        }
        Rule::struct_call => {
            let mut pairs = pair.into_inner();
            let struct_name_pair = pairs.next().unwrap();
            let struct_name = struct_name_pair.as_str();
            let fn_call_pair = pairs.next().unwrap();
            evaluate_function_call(fn_call_pair, storage, Some(struct_name))
        }
        Rule::postfix => {
            let mut inner_pairs = pair.into_inner();
            let first_pair = inner_pairs.next().unwrap();
            let mut value = parse_expr(first_pair, storage)?.unwrap();
            for postfix_pair in inner_pairs {
                value = match postfix_pair.as_rule() {
                    Rule::fac => value.try_factorial()?,
                    Rule::vector_index => {
                        let index = evaluate_index(postfix_pair, storage)?;
                        value.try_vector_index(index)?
                    }
                    //Rule::matrix_index => value.try_matrix_index()?,
                    _ => unreachable!("Parser should only pass valid postfixes"),
                };
            }
            Ok(Some(value))
        }
        Rule::prefix => {
            let mut inner_pairs = pair.into_inner();
            let first = inner_pairs.next().unwrap();

            let value = if first.as_rule() == Rule::neg {
                // When the first token is a negation operator,
                // the next token is the operand.
                let operand = inner_pairs.next().unwrap();
                parse_expr(operand, storage)?.unwrap().try_negative()?
            } else {
                // Otherwise, the first token is the actual operand.
                parse_expr(first, storage)?.unwrap()
            };

            Ok(Some(value))
        }
        Rule::vector => {
            let elements_pair = pair.into_inner().next().unwrap();
            let value_pairs = elements_pair.into_inner();
            let mut values = Vec::new();
            for value_pair in value_pairs {
                if let Some(value) = parse_expr(value_pair, storage)? {
                    values.push(value.as_f64()?);
                } else {
                    return Err(NadirParserErrors::CantParseVector);
                }
            }
            Ok(Some(Value::Vector(Box::new(DVector::from_vec(values)))))
        }

        _ => Err(NadirParserErrors::UnexpectedRule(pair.as_rule())),
    }
}

fn evaluate_function_call(
    pair: Pair<Rule>,
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
            let value = parse_expr(arg, storage)?.ok_or_else(|| NadirParserErrors::EmptyValue)?;
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

fn evaluate_index(
    pair: Pair<Rule>,
    storage: &mut Rc<RefCell<Storage>>,
) -> Result<IndexStyle, NadirParserErrors> {
    let index_pair = pair.into_inner().next().unwrap();
    let index_style_pair = index_pair.into_inner().next().unwrap();
    match index_style_pair.as_rule() {
        Rule::index_all => Ok(IndexStyle::All),
        Rule::index_single => {
            let expr_pair = index_style_pair.into_inner().next().unwrap();
            let value = parse_expr(expr_pair, storage)?.unwrap();
            Ok(value.as_index()?)
        }
        Rule::index_range_inclusive => {
            let mut range_pairs = index_style_pair.into_inner();
            let first = if let Some(first_pair) = range_pairs.next() {
                Some(parse_expr(first_pair, storage)?.unwrap().as_usize()?)
            } else {
                None
            };
            let second = if let Some(second_pair) = range_pairs.next() {
                Some(parse_expr(second_pair, storage)?.unwrap().as_usize()?)
            } else {
                None
            };
            let third = if let Some(third_pair) = range_pairs.next() {
                Some(parse_expr(third_pair, storage)?.unwrap().as_usize()?)
            } else {
                None
            };

            let range = match (first, second, third) {
                (Some(first), Some(second), None) => Range {
                    start: Some(first),
                    stop: Some(second + 1),
                    step: None,
                },
                (Some(first), Some(second), Some(third)) => Range {
                    start: Some(first),
                    stop: Some(third + 1),
                    step: Some(second),
                },
                // just return default range, which should index as all
                _ => Range::default(),
            };
            Ok(IndexStyle::Range(range))
        }
        Rule::index_range_exclusive => {
            let mut range_pairs = index_style_pair.into_inner();
            let first = if let Some(first_pair) = range_pairs.next() {
                Some(parse_expr(first_pair, storage)?.unwrap().as_usize()?)
            } else {
                None
            };
            let second = if let Some(second_pair) = range_pairs.next() {
                Some(parse_expr(second_pair, storage)?.unwrap().as_usize()?)
            } else {
                None
            };
            let third = if let Some(third_pair) = range_pairs.next() {
                Some(parse_expr(third_pair, storage)?.unwrap().as_usize()?)
            } else {
                None
            };

            let range = match (first, second, third) {
                (Some(first), Some(second), None) => Range {
                    start: Some(first),
                    stop: Some(second),
                    step: None,
                },
                (Some(first), Some(second), Some(third)) => Range {
                    start: Some(first),
                    stop: Some(third),
                    step: Some(second),
                },
                // just return default range, which should index as all
                _ => Range::default(),
            };
            Ok(IndexStyle::Range(range))
        }
        _ => {
            dbg!(index_style_pair.as_rule());
            unreachable!("rule needs to be an index")
        }
    }
}
