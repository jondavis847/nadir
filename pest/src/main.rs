use colored::Colorize;
use pest::iterators::{Pair, Pairs};
use pest::pratt_parser::{Assoc, Op, PrattParser};
use pest::Parser;
use pest_derive::Parser;
use rustyline::error::ReadlineError;
use rustyline::DefaultEditor;
use std::fmt::Debug;
use thiserror::Error;

mod clone_any;
mod storage;
mod value;

use storage::Storage;
use value::Value;

#[derive(Parser)]
#[grammar = "main.pest"] // relative path to your .pest file
struct NadirParser;

#[derive(Debug, Error)]
pub enum NadirParserErrors {}

fn main() {
    // `()` can be used when no completer is required
    let mut rl = DefaultEditor::new().expect("Failed to create rustyline editor");

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

    // Custom prompt with bold and colored text
    let prompt = "nadir> ".bold().cyan().to_string();

    // This store holds variables as Box<dyn Any> so we can put any type into it.
    let mut storage = Storage::default();

    let pratt = PrattParser::new()
        .op(Op::infix(Rule::add, Assoc::Left) | Op::infix(Rule::sub, Assoc::Left))
        .op(Op::infix(Rule::mul, Assoc::Left) | Op::infix(Rule::div, Assoc::Left))
        .op(Op::infix(Rule::pow, Assoc::Right))
        .op(Op::prefix(Rule::neg))
        .op(Op::postfix(Rule::fac));

    loop {
        // Display the prompt and read user input
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
                            Ok(pairs) => {
                                parse_expr(pairs, &pratt, &mut storage);
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
    pairs: Pairs<Rule>,
    pratt: &PrattParser<Rule>,
    storage: &mut Storage,
) -> Result<Option<Value>, NadirParserErrors> {
    pratt
        .map_primary(|primary| {
            match primary.as_rule() {
                Rule::number => evaluate_number(primary),
                Rule::identifier => evaluate_identifier(primary, &storage),
                Rule::function_call => unimplemented!("todo"),
                Rule::expression => parse_expr(primary.into_inner(), pratt, storage),
                _ => Ok(None), // For any other rule, return None
            }
        })
        .map_prefix(|op, rhs| {
            rhs.and_then(|rhs_value| {
                match op.as_rule() {
                    Rule::neg => {
                        // Assuming Value::Number and implementing negation
                        if let Value::Number(n) = rhs_value {
                            Ok(Some(Value::Number(-n)))
                        } else {
                            Err(NadirParserErrors::InvalidOperation)
                        }
                    }
                    _ => Err(NadirParserErrors::UnexpectedOperator),
                }
            })
        })
        .map_postfix(|lhs, op| {
            lhs.and_then(|lhs_value| {
                match op.as_rule() {
                    Rule::fac => {
                        // Assuming Value::Number and implementing factorial
                        if let Value::Number(n) = lhs_value {
                            if n >= 0.0 && n.fract() == 0.0 {
                                let result = (1..=n as u64).product::<u64>() as f64;
                                Ok(Some(Value::Number(result)))
                            } else {
                                Err(NadirParserErrors::InvalidNumber)
                            }
                        } else {
                            Err(NadirParserErrors::InvalidOperation)
                        }
                    }
                    _ => Err(NadirParserErrors::UnexpectedOperator),
                }
            })
        })
        .map_infix(|lhs, op, rhs| {
            lhs.and_then(|lhs_value| {
                rhs.and_then(|rhs_value| match (lhs_value, rhs_value) {
                    (Value::Number(lhs_num), Value::Number(rhs_num)) => match op.as_rule() {
                        Rule::add => Ok(Some(Value::Number(lhs_num + rhs_num))),
                        Rule::sub => Ok(Some(Value::Number(lhs_num - rhs_num))),
                        Rule::mul => Ok(Some(Value::Number(lhs_num * rhs_num))),
                        Rule::div => {
                            if rhs_num != 0.0 {
                                Ok(Some(Value::Number(lhs_num / rhs_num)))
                            } else {
                                Err(NadirParserErrors::DivisionByZero)
                            }
                        }
                        Rule::pow => Ok(Some(Value::Number(lhs_num.powf(rhs_num)))),
                        _ => Err(NadirParserErrors::UnexpectedOperator),
                    },
                    _ => Err(NadirParserErrors::InvalidOperation),
                })
            })
        })
        .parse(pairs)
}

fn evaluate_number(primary: Pair<Rule>) -> Result<Option<Value>, NadirParserErrors> {
    let inner_pair = primary
        .into_inner()
        .next()
        .ok_or(NadirParserErrors::InvalidNumber)?;
    match inner_pair.as_rule() {
        Rule::float => inner_pair
            .as_str()
            .parse::<f64>()
            .map(Value::new)
            .map(Some)
            .map_err(|_| NadirParserErrors::InvalidNumber),
        Rule::integer => inner_pair
            .as_str()
            .parse::<i64>()
            .map(Value::new)
            .map(Some)
            .map_err(|_| NadirParserErrors::InvalidNumber),
        _ => Err(NadirParserErrors::InvalidNumber),
    }
}

fn evaluate_identifier(
    primary: Pair<Rule>,
    storage: &Storage,
) -> Result<Option<Value>, NadirParserErrors> {
    Ok(Some(storage.get(primary.as_str())?))
}
