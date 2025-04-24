use std::{
    path::PathBuf,
    sync::{Arc, Mutex},
};

use crate::{
    DaemonToRepl, NadirParser, ReplToDaemon, ReplToSubscription, Rule,
    helper::NadirHelper,
    registry::Registry,
    storage::Storage,
    value::{Enum, Event, IndexStyle, Range, Value},
};
use iced::futures::{
    SinkExt, StreamExt,
    channel::mpsc::{Receiver, Sender},
    executor::block_on,
};
use nalgebra::{DMatrix, DVector};
use pest::{Parser, iterators::Pair};
use rustyline::{CompletionType, Config, EditMode, Editor, error::ReadlineError};

pub mod errors;
use errors::ReplErrors;

#[derive(Default)]
struct ReplChannels {
    repl_to_daemon: Option<Sender<ReplToDaemon>>,
    daemon_to_repl: Option<Receiver<DaemonToRepl>>,
    repl_to_plot_subscription: Option<Sender<ReplToSubscription>>,
}

pub struct NadirRepl {
    ans: Value,
    registry: Arc<Mutex<Registry>>,
    storage: Arc<Mutex<Storage>>,
    pwd: Arc<Mutex<PathBuf>>,
    channels: ReplChannels,
}

impl NadirRepl {
    pub fn new(
        registry: Arc<Mutex<Registry>>,
        storage: Arc<Mutex<Storage>>,
        pwd: Arc<Mutex<PathBuf>>,
    ) -> Self {
        Self {
            ans: Value::None,
            registry,
            storage,
            channels: ReplChannels::default(),
            pwd,
        }
    }

    pub fn connect_plot_daemon(
        &mut self,
        repl_to_daemon: Sender<ReplToDaemon>,
        daemon_to_repl: Receiver<DaemonToRepl>,
    ) {
        self.channels.repl_to_daemon = Some(repl_to_daemon);
        self.channels.daemon_to_repl = Some(daemon_to_repl);
    }

    pub fn run(&mut self) -> Result<(), Box<dyn std::error::Error>> {
        let config = Config::builder()
            .history_ignore_space(true)
            .completion_type(CompletionType::List)
            .edit_mode(EditMode::Emacs)
            .build();
        let h = NadirHelper::new(
            self.registry.clone(),
            self.storage.clone(),
            self.pwd.clone(),
        );

        // `()` can be used when no completer is required
        let mut rl = Editor::with_config(config)?;
        rl.set_helper(Some(h));
        // Determine the configuration directory
        let history_path = if let Some(mut history_path) = dirs::config_dir() {
            history_path.push("nadir");
            if !history_path.exists() {
                std::fs::create_dir_all(&history_path)?;
            }
            history_path.push("cli_history.txt");

            if history_path.exists() {
                #[cfg(feature = "with-file-history")]
                rl.load_history(&history_path)?
            }
            Some(history_path)
        } else {
            None
        };

        let prompt = "nadir> ";

        // wait for the subscription sender from the daemon
        loop {
            if let Some(rx) = &mut self.channels.daemon_to_repl {
                // Just process one message at a time to keep things simple
                if let Some(cmd) = block_on(rx.next()) {
                    match cmd {
                        DaemonToRepl::ReplToSubscriptionTx(tx) => {
                            self.channels.repl_to_plot_subscription = Some(tx);
                            break;
                        } // Handle other message types...
                    }
                }
            }
        }

        loop {
            // Display the prompt and read user input
            rl.helper_mut().expect("No helper").colored_prompt =
                format!("\x1b[38;2;246;189;96m{prompt}\x1b[0m");
            match rl.readline(&prompt) {
                Ok(line) => {
                    // Add the input to history
                    rl.add_history_entry(line.as_str())?;

                    // Handle specific commands or process input
                    match line.trim() {
                        "exit" => break,
                        "vars" => {
                            let storage = self.storage.lock().unwrap();
                            let keys: Vec<&String> = storage.get_names();
                            for name in keys {
                                println!("{}", name);
                            }
                        }
                        "ans" => {
                            println!("{:?}", self.ans);
                        }
                        _ => {
                            // Parse the input using the "line" rule.
                            let mut pairs = NadirParser::parse(Rule::line, &line)?;

                            if let Some(line_pair) = pairs.next() {
                                // dbg!(&line_pair);
                                // get to next level, with is a silent_line or print_line
                                let print_or_silent = line_pair.into_inner().next().unwrap();

                                let value = match self.parse_expr(print_or_silent) {
                                    Ok(v) => v,
                                    Err(e) => {
                                        eprintln!("{e}");
                                        Value::None
                                    }
                                };
                                // If there is some event, perform it
                                match value {
                                    Value::Event(event) => match event {
                                        Event::Animate => {
                                            if let Some(repl_to_subscription) =
                                                &mut self.channels.repl_to_plot_subscription
                                            {
                                                let pwd = std::env::current_dir()?;
                                                block_on(
                                                    repl_to_subscription
                                                        .send(ReplToSubscription::Animate(pwd)),
                                                )?
                                            }
                                        }
                                        Event::NewFigure => {
                                            if let Some(repl_to_subscription) =
                                                &mut self.channels.repl_to_plot_subscription
                                            {
                                                block_on(
                                                    repl_to_subscription
                                                        .send(ReplToSubscription::NewFigure),
                                                )?
                                            }
                                        }
                                        Event::CloseAllFigures => {
                                            if let Some(repl_to_subscription) =
                                                &mut self.channels.repl_to_plot_subscription
                                            {
                                                block_on(
                                                    repl_to_subscription
                                                        .send(ReplToSubscription::CloseAllFigures),
                                                )?
                                            }
                                        }
                                    },
                                    _ => {}
                                }
                            }
                        }
                    }
                }
                Err(ReadlineError::Interrupted) => {
                    // Ctrl+C
                    continue;
                }
                Err(ReadlineError::Eof) => {
                    // Ctrl+D
                    break;
                }
                Err(err) => {
                    // Handle other potential errors
                    println!("Error: {:?}", err);
                    break;
                }
            }
        }

        // loop broke, send a message to ice to close the daemon
        if let Some(channel) = &mut self.channels.repl_to_plot_subscription {
            block_on(channel.send(ReplToSubscription::ReplClosed))?;
        }
        if let Some(history_path) = &history_path {
            #[cfg(feature = "with-file-history")]
            rl.save_history(history_path)
                .expect("Failed to save history");
        }
        Ok(())
    }

    fn parse_expr(&mut self, pair: Pair<Rule>) -> Result<Value, ReplErrors> {
        match pair.as_rule() {
            Rule::additive => {
                let mut inner_pairs = pair.into_inner();
                let mut value = self.parse_expr(inner_pairs.next().unwrap())?;

                while let (Some(op_pair), Some(right_pair)) =
                    (inner_pairs.next(), inner_pairs.next())
                {
                    let right = self.parse_expr(right_pair)?;
                    value = match op_pair.as_rule() {
                        Rule::add => value.try_add(&right)?,
                        Rule::sub => value.try_sub(&right)?,
                        _ => unreachable!(),
                    };
                }

                Ok(value)
            }
            Rule::assignment => {
                let mut inner = pair.into_inner();
                let name = inner.next().unwrap().as_str();
                // check if name is reserved
                match name {
                    "ans" => return Err(ReplErrors::NameReserved("ans")),
                    _ => {}
                }
                let expr = inner.next().unwrap();
                let value = self.parse_expr(expr)?;
                self.storage
                    .lock()
                    .unwrap()
                    .insert(name.to_string(), value.clone())?;

                Ok(value)
            }
            Rule::command => {
                let mut inner = pair.into_inner();
                let mut has_args_pair = inner.next().unwrap().into_inner();
                let cmd_name = has_args_pair.next().unwrap().as_str();
                let mut args = Vec::new();
                while let Some(arg) = has_args_pair.next() {
                    args.push(arg.as_str().to_string())
                }
                // Execute the command
                Ok(self
                    .registry
                    .lock()
                    .unwrap()
                    .eval_command(cmd_name, args, self.pwd.clone())?)
            }
            Rule::comparison => {
                let mut inner_pairs = pair.into_inner();
                // The first additive (e.g. `a`)
                let first_pair = inner_pairs.next().unwrap();
                let mut prev_value = self.parse_expr(first_pair)?;

                // We'll store a running boolean result (chained comparisons).
                let mut result = true;

                while let Some(comp_pair) = inner_pairs.next() {
                    let next_pair = inner_pairs.next().unwrap();
                    let next_value = self.parse_expr(next_pair)?;

                    // Evaluate "prev_value < next_value" or similar
                    let partial = match comp_pair.as_str() {
                        "<" => prev_value.try_lt(&next_value)?,
                        "<=" => prev_value.try_lte(&next_value)?,
                        ">" => prev_value.try_gt(&next_value)?,
                        ">=" => prev_value.try_gte(&next_value)?,
                        _ => unreachable!("comparator requried for parse"),
                    };

                    result = result && partial.as_bool()?; // chain with AND
                    if !result {
                        break; // short-circuit if one comparison fails
                    }

                    // For the next comparison, "next_value" becomes "prev_value"
                    prev_value = next_value;
                }

                Ok(Value::bool(result))
            }
            Rule::enumeration => self.evaluate_enum(pair),
            Rule::expr => {
                let inner_pair = pair.into_inner().next().unwrap();
                self.parse_expr(inner_pair)
            }
            Rule::exponential => {
                let mut inner_pairs = pair.into_inner();
                let mut value = self.parse_expr(inner_pairs.next().unwrap())?;

                while let (Some(op_pair), Some(right_pair)) =
                    (inner_pairs.next(), inner_pairs.next())
                {
                    let right = self.parse_expr(right_pair)?;
                    value = match op_pair.as_rule() {
                        Rule::pow => value.try_pow(&right)?,
                        _ => unreachable!(),
                    };
                }
                Ok(value)
            }
            Rule::float => Ok(Value::f64(pair.as_str().parse::<f64>().unwrap())),
            Rule::function_call => self.evaluate_function_call(pair, None),
            Rule::identifier => {
                let ident = pair.as_str();
                match ident {
                    "ans" => Ok(self.ans.clone()),
                    _ => Ok(self.storage.lock().unwrap().get(pair.as_str())?),
                }
            }
            Rule::instance_field => self.evaluate_instance_field(pair),
            Rule::instance_call => self.evaluate_instance_call(pair),
            Rule::integer => Ok(Value::i64(pair.as_str().parse::<i64>().unwrap())),
            Rule::matrix => {
                let row_pairs = pair.into_inner();
                let mut rows: Vec<Vec<f64>> = Vec::new();
                // Parse each row from pest.
                for row_pair in row_pairs {
                    let space_separated_pair = row_pair.into_inner().next().unwrap();
                    let mut row: Vec<f64> = Vec::new();
                    for value_pair in space_separated_pair.into_inner() {
                        let value = self.parse_expr(value_pair)?;
                        row.push(value.as_f64()?);
                    }
                    rows.push(row);
                }

                // Check that we have at least one row and that all rows are of equal length.
                if rows.is_empty() {
                    return Err(ReplErrors::CantParseMatrix);
                }
                let row_len = rows[0].len();
                if !rows.iter().all(|r| r.len() == row_len) {
                    return Err(ReplErrors::MatrixRowLengthMismatch);
                }

                // Flatten the matrix in row-major order.
                let flat_data: Vec<f64> = rows.into_iter().flatten().collect();
                let num_rows = flat_data.len() / row_len;

                // Create the DMatrix using from_row_slice which accepts row-major data.
                let matrix = DMatrix::from_row_slice(num_rows, row_len, &flat_data);

                Ok(Value::Matrix(Arc::new(Mutex::new(matrix))))
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
                    Some(self.parse_expr(row_index_pair)?.as_usize()?)
                };

                let col_selection = if col_index_pair.as_str() == ":" {
                    None // None signifies full column selection
                } else {
                    Some(self.parse_expr(col_index_pair)?.as_usize()?)
                };

                let matrix = match self.parse_expr(matrix_pair)? {
                    Value::Matrix(matrix) => matrix,
                    _ => unreachable!("shouldn't be here if it's not a matrix"),
                };
                let matrix = &*matrix.lock().unwrap();
                // Extract the desired elements based on the selection
                match (row_selection, col_selection) {
                    (Some(row), Some(col)) => {
                        // Specific element
                        if row > matrix.nrows() {
                            return Err(ReplErrors::OutOfBoundsIndex(
                                row.to_string(),
                                matrix.nrows().to_string(),
                            ));
                        }
                        if col > matrix.ncols() {
                            return Err(ReplErrors::OutOfBoundsIndex(
                                col.to_string(),
                                matrix.ncols().to_string(),
                            ));
                        }
                        let value = matrix.get((row, col)).unwrap();
                        Ok(Value::f64(*value))
                    }
                    (Some(row), None) => {
                        // Entire row
                        let row_vec = matrix.row(row).transpose().into_owned();
                        Ok(Value::Vector(Arc::new(Mutex::new(row_vec))))
                    }
                    (None, Some(col)) => {
                        // Entire column
                        let col_vec = matrix.column(col).into_owned();
                        Ok(Value::Vector(Arc::new(Mutex::new(col_vec))))
                    }
                    (None, None) => {
                        // Entire matrix
                        Ok(Value::Matrix(Arc::new(Mutex::new(matrix.clone()))))
                    }
                }
            }
            Rule::multiplicative => {
                let mut inner_pairs = pair.into_inner();
                let mut value = self.parse_expr(inner_pairs.next().unwrap())?;

                while let (Some(op_pair), Some(right_pair)) =
                    (inner_pairs.next(), inner_pairs.next())
                {
                    let right = self.parse_expr(right_pair)?;
                    value = match op_pair.as_rule() {
                        Rule::mul => value.try_mul(&right)?,
                        Rule::div => value.try_div(&right)?,
                        _ => unreachable!(),
                    };
                }

                Ok(value)
            }
            Rule::print_line => {
                let next_pair = pair.into_inner().next().unwrap();
                let value = self.parse_expr(next_pair)?;
                self.ans = value.clone();
                println!("{:?}", value);
                Ok(value)
            }
            Rule::silent_line => {
                let next_pair = pair.into_inner().next().unwrap();
                let value = self.parse_expr(next_pair)?;
                Ok(value)
            }
            Rule::string => {
                let parsed_str = pair.as_str();
                let unquoted_str = &parsed_str[1..parsed_str.len() - 1]; // Remove the first and last character (the quotes)
                Ok(Value::String(Arc::new(Mutex::new(
                    unquoted_str.to_string(),
                ))))
            }
            Rule::struct_call => {
                let mut pairs = pair.into_inner();
                let struct_name_pair = pairs.next().unwrap();
                let struct_name = struct_name_pair.as_str();
                let fn_call_pair = pairs.next().unwrap();
                self.evaluate_function_call(fn_call_pair, Some(struct_name))
            }
            Rule::postfix => {
                let mut inner_pairs = pair.into_inner();
                let first_pair = inner_pairs.next().unwrap();
                let mut value = self.parse_expr(first_pair)?;
                for postfix_pair in inner_pairs {
                    value = match postfix_pair.as_rule() {
                        Rule::fac => value.try_factorial()?,
                        Rule::vector_index => {
                            let index_pair = postfix_pair.into_inner().next().unwrap();
                            let index = self.evaluate_index(index_pair)?;
                            value.try_vector_index(index)?
                        }
                        Rule::matrix_index => {
                            let mut index_pairs = postfix_pair.into_inner();
                            let first = index_pairs.next().unwrap();
                            let second = index_pairs.next().unwrap();
                            let first_index = self.evaluate_index(first)?;
                            let second_index = self.evaluate_index(second)?;
                            value.try_matrix_index(first_index, second_index)?
                        }
                        //Rule::matrix_index => value.try_matrix_index()?,
                        _ => unreachable!("Parser should only pass valid postfixes"),
                    };
                }
                Ok(value)
            }
            Rule::prefix => {
                let mut inner_pairs = pair.into_inner();
                let first = inner_pairs.next().unwrap();

                let value = if first.as_rule() == Rule::neg {
                    // When the first token is a negation operator,
                    // the next token is the operand.
                    let operand = inner_pairs.next().unwrap();
                    self.parse_expr(operand)?.try_negative()?
                } else {
                    // Otherwise, the first token is the actual operand.
                    self.parse_expr(first)?
                };

                Ok(value)
            }
            Rule::vector => {
                let elements_pair = pair.into_inner().next().unwrap();
                let value_pairs = elements_pair.into_inner();
                let mut values = Vec::new();
                for value_pair in value_pairs {
                    let value = self.parse_expr(value_pair)?;
                    values.push(value.as_f64()?);
                }
                Ok(Value::Vector(Arc::new(Mutex::new(DVector::from_vec(
                    values,
                )))))
            }

            _ => Err(ReplErrors::UnexpectedRule(pair.as_rule())),
        }
    }

    fn evaluate_function_call(
        &mut self,
        pair: Pair<Rule>,
        struct_name: Option<&str>,
    ) -> Result<Value, ReplErrors> {
        let mut pairs = pair.into_inner();

        let fn_name_pair = pairs.next().unwrap();
        let fn_name = fn_name_pair.as_str();

        let args_pair = pairs.next();

        // Process each argument
        let args = if let Some(args_pair) = args_pair {
            let inner = args_pair.into_inner();
            let mut args = Vec::new();
            for arg in inner {
                let value = self.parse_expr(arg)?;
                match value {
                    Value::None => return Err(ReplErrors::EmptyValue),
                    _ => {}
                }
                args.push(value);
            }
            args
        } else {
            Vec::new()
        };

        if let Some(struct_name) = struct_name {
            Ok(self
                .registry
                .lock()
                .unwrap()
                .eval_struct_method(struct_name, fn_name, args)?)
        } else {
            Ok(self.registry.lock().unwrap().eval_function(fn_name, args)?)
        }
    }

    fn evaluate_instance_field(&mut self, pair: Pair<Rule>) -> Result<Value, ReplErrors> {
        let storage = self.storage.lock().unwrap();
        let mut pairs = pair.into_inner();
        let instance_name = pairs.next().unwrap().as_str();
        let instance = storage.get(instance_name)?;
        let field = pairs.next().unwrap().as_str();
        match &instance {
            Value::Quaternion(q) => {
                let q = q.lock().unwrap();
                match field {
                    "x" => Ok(Value::f64(q.x)),
                    "y" => Ok(Value::f64(q.y)),
                    "z" => Ok(Value::f64(q.z)),
                    "w" => Ok(Value::f64(q.w)),
                    _ => Err(ReplErrors::InvalidField(
                        instance_name.to_string(),
                        field.to_string(),
                        instance.to_string(),
                    )),
                }
            }
            Value::UnitQuaternion(q) => {
                let q = q.lock().unwrap();
                match field {
                    "x" => Ok(Value::f64(q.0.x)),
                    "y" => Ok(Value::f64(q.0.y)),
                    "z" => Ok(Value::f64(q.0.z)),
                    "w" => Ok(Value::f64(q.0.w)),
                    _ => Err(ReplErrors::InvalidField(
                        instance_name.to_string(),
                        field.to_string(),
                        instance.to_string(),
                    )),
                }
            }
            Value::Map(map) => {
                let map = &*map.lock().unwrap();
                for (key, value) in &map.0 {
                    if field == key {
                        return Ok(value.clone());
                    }
                }
                Err(ReplErrors::InvalidField(
                    instance_name.to_string(),
                    field.to_string(),
                    instance.to_string(),
                ))
            }
            _ => Err(ReplErrors::InvalidField(
                instance_name.to_string(),
                field.to_string(),
                instance.to_string(),
            )),
        }
    }

    fn evaluate_index(&mut self, pair: Pair<Rule>) -> Result<IndexStyle, ReplErrors> {
        let index_style_pair = pair.into_inner().next().unwrap();

        match index_style_pair.as_rule() {
            Rule::index_all => Ok(IndexStyle::All),
            Rule::index_single => {
                let expr_pair = index_style_pair.into_inner().next().unwrap();
                let value = self.parse_expr(expr_pair)?;
                Ok(value.as_index()?)
            }
            Rule::index_range_inclusive => {
                let mut range_pairs = index_style_pair.into_inner();
                let first = if let Some(first_pair) = range_pairs.next() {
                    Some(self.parse_expr(first_pair)?.as_usize()?)
                } else {
                    None
                };
                let second = if let Some(second_pair) = range_pairs.next() {
                    Some(self.parse_expr(second_pair)?.as_usize()?)
                } else {
                    None
                };
                let third = if let Some(third_pair) = range_pairs.next() {
                    Some(self.parse_expr(third_pair)?.as_usize()?)
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
                    Some(self.parse_expr(first_pair)?.as_usize()?)
                } else {
                    None
                };
                let second = if let Some(second_pair) = range_pairs.next() {
                    Some(self.parse_expr(second_pair)?.as_usize()?)
                } else {
                    None
                };
                let third = if let Some(third_pair) = range_pairs.next() {
                    Some(self.parse_expr(third_pair)?.as_usize()?)
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

    fn evaluate_instance_call(&mut self, pair: Pair<Rule>) -> Result<Value, ReplErrors> {
        let mut pairs = pair.into_inner();
        let instance_name = pairs.next().unwrap().as_str();
        let instance = self.storage.lock().unwrap().get(instance_name)?;
        let struct_name = instance.to_string();
        let mut function_pairs = pairs.next().unwrap().into_inner();
        let method_name = function_pairs.next().unwrap().as_str();

        // Process each argument
        let args_pair = function_pairs.next();
        let args = if let Some(args_pair) = args_pair {
            let inner = args_pair.into_inner();
            let mut args = Vec::new();
            for arg in inner {
                let value = self.parse_expr(arg)?;
                match value {
                    Value::None => return Err(ReplErrors::EmptyValue),
                    _ => {}
                }
                args.push(value);
            }
            args
        } else {
            Vec::new()
        };

        Ok(self.registry.lock().unwrap().eval_instance_method(
            instance,
            &struct_name,
            method_name,
            args,
        )?)
    }

    fn evaluate_enum(&mut self, pair: Pair<Rule>) -> Result<Value, ReplErrors> {
        let mut inner_pairs = pair.into_inner();
        let name = inner_pairs.next().unwrap().as_str();
        let variant = inner_pairs.next().unwrap().as_str();
        match (name, variant) {
            ("CelestialBodies", "Earth")
            | ("CelestialBodies", "Jupiter")
            | ("CelestialBodies", "Mars")
            | ("CelestialBodies", "Mercury")
            | ("CelestialBodies", "Moon")
            | ("CelestialBodies", "Neptune")
            | ("CelestialBodies", "Pluto")
            | ("CelestialBodies", "Saturn")
            | ("CelestialBodies", "Sun")
            | ("CelestialBodies", "Uranus")
            | ("CelestialBodies", "Venus")
            | ("TimeSystem", "GPS")
            | ("TimeSystem", "TAI")
            | ("TimeSystem", "UTC")
            | ("TimeSystem", "TDB") => Ok(Value::Enum(Enum {
                name: name.to_string(),
                variant: variant.to_string(),
            })),
            _ => Err(ReplErrors::InvalidEnum(
                name.to_string(),
                variant.to_string(),
            )),
        }
    }
}
