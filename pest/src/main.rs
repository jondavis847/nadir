use colored::Colorize;
use pest::iterators::Pair;
use pest::Parser;
use pest_derive::Parser;
use rustyline::error::ReadlineError;
use rustyline::DefaultEditor;
use std::any::{type_name, Any, TypeId};
use std::collections::HashMap;
use std::fmt::Debug;

pub trait AnyClone: Any + Debug {
    fn clone_box(&self) -> Box<dyn AnyClone>;
    fn as_any(&self) -> &dyn Any;

    fn as_any_mut(&mut self) -> &mut dyn Any;
}

impl<T> AnyClone for T
where
    T: Clone + Any + Debug + 'static,
{
    fn clone_box(&self) -> Box<dyn AnyClone> {
        Box::new(self.clone())
    }

    fn as_any(&self) -> &dyn Any {
        self
    }

    fn as_any_mut(&mut self) -> &mut dyn Any {
        self
    }
}

impl dyn AnyClone {
    /// Allow downcasting `Box<dyn AnyClone>` to the original type
    pub fn downcast<T: 'static>(self: Box<Self>) -> Result<Box<T>, Box<Self>> {
        if (*self).as_any().type_id() == TypeId::of::<T>() {
            // SAFETY: We just checked that the type is correct
            let raw: *mut dyn AnyClone = Box::into_raw(self);
            Ok(unsafe { Box::from_raw(raw as *mut T) })
        } else {
            Err(self)
        }
    }

    /// Allow getting a reference to the original type
    pub fn downcast_ref<T: 'static>(&self) -> Option<&T> {
        self.as_any().downcast_ref::<T>()
    }

    /// Allow getting a mutable reference to the original type
    pub fn downcast_mut<T: 'static>(&mut self) -> Option<&mut T> {
        self.as_any_mut().downcast_mut::<T>()
    }
}
pub struct Value {
    value: Box<dyn AnyClone>,
    type_id: TypeId,
    type_name: &'static str,
}

impl Value {
    /// Create a new `Value` instance
    pub fn new<T: AnyClone>(value: T) -> Self {
        Self {
            value: Box::new(value),
            type_name: type_name::<T>(),
            type_id: TypeId::of::<T>(),
        }
    }

    /// Attempt to downcast the `Value` to a specific type
    pub fn downcast<T: 'static>(self) -> Result<T, Self> {
        if self.type_id == TypeId::of::<T>() {
            Ok(*self.value.downcast::<T>().unwrap())
        } else {
            Err(self)
        }
    }

    /// Attempt to get a reference to the value as a specific type
    pub fn as_ref<T: 'static>(&self) -> Option<&T> {
        if self.type_id == TypeId::of::<T>() {
            self.value.as_ref().downcast_ref::<T>()
        } else {
            None
        }
    }
}

impl Clone for Value {
    fn clone(&self) -> Self {
        Self {
            value: self.value.clone_box(),
            type_id: self.type_id,
            type_name: self.type_name,
        }
    }
}

impl std::fmt::Debug for Value {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        // Assuming `self.type_name` returns the type name as a &str
        let type_name = self.type_name; // Replace with your actual method to get the type name
        write!(f, "{} ", type_name.bold().green())?;
        self.value.fmt(f)
    }
}

#[derive(Parser)]
#[grammar = "main.pest"] // relative path to your .pest file
struct NadirParser;

#[derive(Debug)]
pub enum NadirParserErrors {
    EnclosedInnerNotExpression,
    EmptyExpression,
    IdentifierNotFound,
    InvalidNumber,
    MissingOperand,
    MissingOperator,
    UnexpectedOperator,
    UnexpectedRule,
}

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
    let mut store: HashMap<String, Value> = HashMap::new();

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
                        let mut keys: Vec<&String> = store.keys().collect();
                        keys.sort();
                        for name in keys {
                            println!("{}", name);
                        }
                    }
                    _ => {
                        // Parse the input using the "line" rule.
                        let parse_result = NadirParser::parse(Rule::line, &line);
                        match parse_result {
                            Ok(pairs) => {
                                for pair in pairs {
                                    match evaluate_line(pair, &mut store) {
                                        Ok(_) => {}
                                        Err(e) => {
                                            eprintln!("Evaluation error: {:?}", e);
                                        }
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

fn evaluate_line(
    pair: Pair<Rule>,
    store: &mut HashMap<String, Value>,
) -> Result<(), NadirParserErrors> {
    let mut pairs = pair.into_inner();
    let pair = pairs.next().unwrap();
    match pair.as_rule() {
        Rule::silent_line => evaluate_silent_line(pair, store)?,
        Rule::print_line => evaluate_print_line(pair, store)?,
        _ => unreachable!("Unexpected rule"),
    }
    Ok(())
}

fn evaluate_silent_line(
    pair: Pair<Rule>,
    store: &mut HashMap<String, Value>,
) -> Result<(), NadirParserErrors> {
    let mut pairs = pair.into_inner();
    let pair = pairs.next().unwrap();
    match pair.as_rule() {
        Rule::expression => {
            evaluate_expression(pair, store, false)?;
        }
        Rule::assignment => evaluate_assignment(pair, store, false)?,
        _ => unreachable!("Unexpected rule"),
    }
    Ok(())
}

fn evaluate_print_line(
    pair: Pair<Rule>,
    store: &mut HashMap<String, Value>,
) -> Result<(), NadirParserErrors> {
    let mut pairs = pair.into_inner();
    let pair = pairs.next().unwrap();
    match pair.as_rule() {
        Rule::expression => {
            evaluate_expression(pair, store, true)?;
        }
        Rule::assignment => {
            evaluate_assignment(pair, store, true)?;
        }
        _ => unreachable!("Unexpected rule"),
    }
    Ok(())
}

fn evaluate_assignment(
    pair: Pair<Rule>,
    store: &mut HashMap<String, Value>,
    print: bool,
) -> Result<(), NadirParserErrors> {
    let mut pairs = pair.into_inner();
    if let Some(identifier) = pairs.next() {
        let identifier = identifier.as_str().to_string();
        if let Some(expression) = pairs.next() {
            let value = evaluate_expression(expression, store, false)?;
            store.insert(identifier.clone(), value.clone());
            if print {
                println!("{} = {:?}", identifier, value);
            }
        }
    }
    Ok(())
}

fn evaluate_expression(
    pair: Pair<Rule>,
    store: &mut HashMap<String, Value>,
    print: bool,
) -> Result<Value, NadirParserErrors> {
    // Loop through the sub-rules of the expression
    let mut pairs = pair.into_inner(); // `into_inner` gives access to the sub-rules
    let mut result: Option<Value> = None;

    while let Some(sub_pair) = pairs.next() {
        // Evaluate each sub-rule recursively
        let value = match sub_pair.as_rule() {
            Rule::enclosed => {
                let inner_pair = sub_pair
                    .into_inner()
                    .next()
                    .ok_or(NadirParserErrors::EnclosedInnerNotExpression)?;
                evaluate_expression(inner_pair, store, false)?
            }

            Rule::identifier => {
                let identifier = sub_pair.as_str();
                if let Some(value) = store.get(identifier) {
                    value.clone()
                } else {
                    return Err(NadirParserErrors::IdentifierNotFound);
                }
            }

            Rule::number => {
                let number: f64 = sub_pair
                    .as_str()
                    .parse()
                    .map_err(|_| NadirParserErrors::InvalidNumber)?;
                Value::new(number) // Assuming `Value::new` creates a `Value` from a number
            }

            Rule::function_call => {
                unimplemented!("Function calls are not implemented yet");
                //evaluate_function_call(sub_pair, store)? // Delegate function call evaluation
            }
            Rule::math => {
                unimplemented!("math not implemented yet");
                // For math operations, we need the left and right operands and the operator
                // let mut inner_pairs = sub_pair.into_inner();
                // let left = evaluate_expression(
                //     inner_pairs
                //         .next()
                //         .ok_or(NadirParserErrors::MissingOperand)?,
                //     store,
                //     false,
                // )?;
                // let operator = inner_pairs
                //     .next()
                //     .ok_or(NadirParserErrors::MissingOperator)?
                //     .as_str();
                // let right = evaluate_expression(
                //     inner_pairs
                //         .next()
                //         .ok_or(NadirParserErrors::MissingOperand)?,
                //     store,
                //     false,
                // )?;

                // // Perform the operation
                // match operator {
                //     "+" => Value::new(left.as_f64()? + right.as_f64()?),
                //     "-" => Value::new(left.as_f64()? - right.as_f64()?),
                //     "*" => Value::new(left.as_f64()? * right.as_f64()?),
                //     "/" => {
                //         let divisor = right.as_f64()?;
                //         Value::new(left.as_f64()? / divisor)
                //     }
                //     _ => return Err(NadirParserErrors::UnexpectedOperator),
                // }
            }

            _ => return Err(NadirParserErrors::UnexpectedRule),
        };

        // Combine results if necessary, or use the first evaluated value
        result = Some(value);
    }

    // If the result is still None, it means we couldn't evaluate anything
    let final_result = result.ok_or(NadirParserErrors::EmptyExpression)?;

    // Optionally print the final result
    if print {
        println!("{:?}", final_result);
    }

    Ok(final_result)
}
