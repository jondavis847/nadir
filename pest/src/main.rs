use pest::iterators::{Pair, Pairs};
use pest::pratt_parser::{Assoc, Op, PrattParser};
use pest::Parser;
use pest_derive::Parser;
use rotations::prelude::Quaternion;
use rustyline::completion::FilenameCompleter;
use rustyline::error::ReadlineError;
use rustyline::highlight::{CmdKind, Highlighter, MatchingBracketHighlighter};
use rustyline::hint::HistoryHinter;
use rustyline::validate::MatchingBracketValidator;
use rustyline::{CompletionType, Config, EditMode, Editor};
use rustyline_derive::{Completer, Helper, Hinter, Validator};
use std::borrow::Cow::{self, Borrowed, Owned};
use std::fmt::Debug;
use thiserror::Error;

mod clone_any;
mod storage;
mod value;

use storage::{Storage, StorageErrors};
use value::{Value, ValueErrors};

#[derive(Parser)]
#[grammar = "main.pest"] // relative path to your .pest file
struct NadirParser;

#[derive(Debug, Error)]
pub enum NadirParserErrors {
    #[error("argument value was None  {0}")]
    ArgumentValueIsNone(String),
    #[error("pest parsing pairs were empty")]
    EmptyPairs(String),
    #[error("expected rule was function_arguments but got {0}")]
    ExpectedFunctionArguments(String),
    #[error("expected rule was function_call but got {0}")]
    ExpectedFunctionCall(String),
    #[error("expected rule was identifier but got {0}")]
    ExpectedIdentifier(String),
    #[error("function {0} not found")]
    FunctionNotFound(String),
    #[error("invalid number {0}")]
    InvalidNumber(String),
    #[error("was not able to parse function arguments")]
    MissingFunctionArguments,
    #[error("was not able to parse a function name")]
    MissingFunctionName,
    #[error("incorrect number of args to function {0}. expected {1}, got {2}")]
    NumberOfArgs(String, String, String),
    #[error("StorageErrors: {0}")]
    StorageErrors(#[from] StorageErrors),
    #[error("unexpected operator {0}")]
    UnexpectedOperator(String),
    #[error("ValueErrors: {0}")]
    ValueErrors(#[from] ValueErrors),
}

fn main() {
    let config = Config::builder()
        .history_ignore_space(true)
        .completion_type(CompletionType::List)
        .edit_mode(EditMode::Emacs)
        .build();
    let h = MyHelper {
        completer: FilenameCompleter::new(),
        highlighter: MatchingBracketHighlighter::new(),
        hinter: HistoryHinter::new(),
        colored_prompt: "".to_owned(),
        validator: MatchingBracketValidator::new(),
    };
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

    // This store holds variables as Box<dyn Any> so we can put any type into it.
    let mut storage = Storage::default();

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
    storage: &mut Storage,
) -> Result<Option<Value>, NadirParserErrors> {
    let pairs_clone = pairs.clone();

    pratt
        .map_primary(|primary| {
            match primary.as_rule() {
                Rule::number => evaluate_number(primary),
                Rule::identifier => evaluate_identifier(primary, &storage),
                Rule::function_call => evaluate_function_call(pairs, pratt, storage),
                //Rule::struct_call => evaluate_struct_call(primary, &storage),
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
        .parse(pairs_clone)
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
    storage: &Storage,
) -> Result<Option<Value>, NadirParserErrors> {
    Ok(Some(storage.get(primary.as_str())?))
}

fn evaluate_function_call(
    pairs: &mut Pairs<Rule>,
    pratt: &PrattParser<Rule>,
    storage: &mut Storage,
) -> Result<Option<Value>, NadirParserErrors> {
    // Extract the function_call pair
    let function_call_pair = pairs
        .next()
        .ok_or_else(|| NadirParserErrors::MissingFunctionName)?;

    // Ensure the pair is indeed a function_call
    if function_call_pair.as_rule() != Rule::function_call {
        return Err(NadirParserErrors::ExpectedFunctionCall(
            function_call_pair.as_str().to_string(),
        ));
    }

    // Iterate over the inner pairs to find identifier and function_arguments
    let mut inner_pairs = function_call_pair.into_inner();
    let fn_name_pair = inner_pairs
        .next()
        .ok_or_else(|| NadirParserErrors::MissingFunctionName)?;
    if fn_name_pair.as_rule() != Rule::identifier {
        return Err(NadirParserErrors::ExpectedIdentifier(
            fn_name_pair.as_str().to_string(),
        ));
    }
    let fn_name = fn_name_pair.as_str();

    // Extract the function arguments
    let args_pair = inner_pairs
        .next()
        .ok_or_else(|| NadirParserErrors::MissingFunctionArguments)?;
    if args_pair.as_rule() != Rule::function_arguments {
        return Err(NadirParserErrors::ExpectedFunctionArguments(
            args_pair.as_str().to_string(),
        ));
    }

    // Process each argument
    let inner = args_pair.into_inner();
    let mut args = Vec::new();
    for arg in inner {
        let arg_clone = arg.clone();
        let value = parse_expr(&mut arg_clone.into_inner(), pratt, storage)?
            .ok_or_else(|| NadirParserErrors::ArgumentValueIsNone(arg.as_str().to_string()))?;
        args.push(value);
    }

    // Match the function name and evaluate
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

// fn evaluate_struct_call(
//     primary: Pair<Rule>,
//     storage: &Storage,
// ) -> Result<Option<Value>, NadirParserErrors> {
// }

#[derive(Helper, Completer, Hinter, Validator)]
struct MyHelper {
    #[rustyline(Completer)]
    completer: FilenameCompleter,
    highlighter: MatchingBracketHighlighter,
    #[rustyline(Validator)]
    validator: MatchingBracketValidator,
    #[rustyline(Hinter)]
    hinter: HistoryHinter,
    colored_prompt: String,
}

impl Highlighter for MyHelper {
    fn highlight_prompt<'b, 's: 'b, 'p: 'b>(
        &'s self,
        prompt: &'p str,
        default: bool,
    ) -> Cow<'b, str> {
        if default {
            Borrowed(&self.colored_prompt)
        } else {
            Borrowed(prompt)
        }
    }

    fn highlight_hint<'h>(&self, hint: &'h str) -> Cow<'h, str> {
        Owned("\x1b[1m".to_owned() + hint + "\x1b[m")
    }

    fn highlight<'l>(&self, line: &'l str, pos: usize) -> Cow<'l, str> {
        self.highlighter.highlight(line, pos)
    }

    fn highlight_char(&self, line: &str, pos: usize, kind: CmdKind) -> bool {
        self.highlighter.highlight_char(line, pos, kind)
    }
}
