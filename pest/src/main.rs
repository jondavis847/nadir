use pest::Parser;
use pest_derive::Parser;
use std::any::Any;
use std::collections::HashMap;
use std::io::{self, Write};

#[derive(Parser)]
#[grammar = "main.pest"] // relative path to your .pest file
struct NadirParser;

fn main() {
    // This store holds variables as Box<dyn Any> so we can put any type into it.
    let mut store: HashMap<String, Box<dyn Any>> = HashMap::new();

    loop {
        // Print a prompt and flush stdout.
        print!("nadir> ");
        io::stdout().flush().unwrap();

        // Read a line from the user.
        let mut input = String::new();
        if io::stdin().read_line(&mut input).unwrap() == 0 {
            // EOF encountered.
            break;
        }

        // Trim the input and check for exit.
        let input = input.trim();
        if input.is_empty() {
            continue;
        }
        if input.eq_ignore_ascii_case("exit") {
            break;
        }

        // Parse the input using the "assignment" rule.
        let parse_result = NadirParser::parse(Rule::line, input);
        match parse_result {
            Ok(mut pairs) => {
                // The top-level rule is "line", which can be either an assignment or a retrieval.
                let pair = pairs.next().unwrap();
                match pair.as_rule() {
                    Rule::assignment => {
                        // For an assignment, get its inner pairs.
                        let mut inner = pair.into_inner();
                        // The first inner pair is the variable name.
                        let var_name = inner.next().unwrap().as_str();
                        // The second inner pair is either an array or a number.
                        let value_pair = inner.next().unwrap();
                        match value_pair.as_rule() {
                            Rule::array => {
                                // For an array, extract the numbers.
                                let elements: Vec<i32> = value_pair
                                    .into_inner()
                                    .filter(|p| p.as_rule() == Rule::number)
                                    .map(|p| p.as_str().parse().expect("Invalid number"))
                                    .collect();
                                println!("Parsed assignment: {} = {:?}", var_name, elements);
                                store.insert(var_name.to_string(), Box::new(elements));
                            }
                            Rule::number => {
                                // Get the inner pair, which will be either a float or an integer.
                                let inner_pair = value_pair.into_inner().next().unwrap();
                                match inner_pair.as_rule() {
                                    Rule::float => {
                                        let num: f64 =
                                            inner_pair.as_str().parse().expect("Invalid float");
                                        println!(
                                            "Parsed assignment: {} = (float) {}",
                                            var_name, num
                                        );
                                        store.insert(var_name.to_string(), Box::new(num));
                                    }
                                    Rule::integer => {
                                        let num: i32 =
                                            inner_pair.as_str().parse().expect("Invalid integer");
                                        println!(
                                            "Parsed assignment: {} = (integer) {}",
                                            var_name, num
                                        );
                                        store.insert(var_name.to_string(), Box::new(num));
                                    }
                                    _ => unreachable!("Unexpected sub-rule in number"),
                                }
                            }
                            _ => unreachable!("Unexpected rule in assignment"),
                        }
                    }
                    Rule::retrieval => {
                        // For a retrieval, the entire input is just an identifier.
                        let var_name = pair.as_str();
                        if let Some(value) = store.get(var_name) {
                            // Attempt to downcast to an integer.
                            if let Some(num) = value.downcast_ref::<i32>() {
                                println!("{num}  :i32");
                            } else if let Some(num) = value.downcast_ref::<f64>() {
                                println!("{num}  :f64");
                            } else if let Some(array) = value.downcast_ref::<Vec<i32>>() {
                                println!("Retrieved {} = {:?}", var_name, array);
                            } else {
                                println!("Retrieved {}: unknown type", var_name);
                            }
                        } else {
                            println!("Variable '{}' not found.", var_name);
                        }
                    }
                    _ => unreachable!("Unexpected top-level rule"),
                }
            }
            Err(e) => {
                eprintln!("Parsing error: {}", e);
            }
        }
    }
}
