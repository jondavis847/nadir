use rustyline::Context;
use rustyline::completion::Completer;
use rustyline::error::ReadlineError;
use rustyline::highlight::{CmdKind, Highlighter, MatchingBracketHighlighter};
use rustyline::validate::MatchingBracketValidator;
use rustyline_derive::{Completer, Helper, Hinter, Validator};
use std::borrow::Cow::{self, Borrowed, Owned};
use std::sync::{Arc, Mutex};

use crate::registry::Registry;
use crate::storage::Storage;

#[derive(Helper, Completer, Hinter, Validator)]
pub struct NadirHelper {
    #[rustyline(Completer)]
    completer: FunctionCompleter,
    highlighter: MatchingBracketHighlighter,
    #[rustyline(Validator)]
    validator: MatchingBracketValidator,
    pub colored_prompt: String,
}

impl NadirHelper {
    pub fn new(registry: Arc<Mutex<Registry>>, storage: Arc<Mutex<Storage>>) -> Self {
        NadirHelper {
            completer: FunctionCompleter::new(registry, storage),
            highlighter: MatchingBracketHighlighter::new(),
            colored_prompt: "".to_owned(),
            validator: MatchingBracketValidator::new(),
        }
    }
}
impl Highlighter for NadirHelper {
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

pub struct FunctionCompleter {
    registry: Arc<Mutex<Registry>>,
    storage: Arc<Mutex<Storage>>,
}

impl FunctionCompleter {
    pub fn new(registry: Arc<Mutex<Registry>>, storage: Arc<Mutex<Storage>>) -> Self {
        Self { registry, storage }
    }

    // /// Helper to complete function names
    // fn complete_enums(prefix: &str) -> Vec<&'static str> {
    //     Self::ENUMS
    //         .iter()
    //         .filter(|func| func.starts_with(prefix))
    //         .cloned()
    //         .collect()
    // }

    // /// Helper to complete function names
    // fn complete_functions(prefix: &str) -> Vec<&'static str> {
    //     Self::FUNCTIONS
    //         .iter()
    //         .filter(|func| func.starts_with(prefix))
    //         .cloned()
    //         .collect()
    // }

    /// Helper to complete struct names
    fn complete_structs(&self, prefix: &str) -> Vec<&'static str> {
        self.registry
            .lock()
            .unwrap()
            .get_structs()
            .iter()
            .filter(|s| s.starts_with(prefix))
            .cloned()
            .collect()
    }

    /// Helper to complete variable names based on prefix
    fn complete_vars(&self, prefix: &str) -> Vec<String> {
        let storage = self.storage.lock().unwrap();
        let vars = storage.get_names();
        vars.iter()
            .filter(|&s| s.starts_with(prefix))
            .map(|s| s.to_string())
            .collect()
    }

    /// Helper to complete struct methods (e.g., "StructName::method")
    fn complete_struct_methods(&self, prefix: &str) -> Vec<String> {
        // Split the prefix on the first "::" delimiter.
        let (type_name, method_prefix) = match prefix.split_once("::") {
            Some((t, m)) => (t, m),
            None => return Vec::new(),
        };

        // Look up the struct information in the registry.
        let registry = self.registry.lock().unwrap();
        let struc = match registry.structs.get(type_name) {
            Some(info) => info,
            None => return Vec::new(),
        };

        // Use iterator combinators to filter and map the struct methods into completions.
        struc
            .struct_methods
            .iter()
            .filter(|(name, _)| name.starts_with(method_prefix))
            .flat_map(|(name, overloads)| {
                overloads.iter().map(move |method| {
                    // Build the argument string, e.g. "rows:i64, cols:i64".
                    let args = method
                        .args
                        .iter()
                        .map(|arg| format!("{}:{}", arg.name, crate::value::label(arg.type_name)))
                        .collect::<Vec<_>>()
                        .join(", ");
                    // Assemble the complete method signature.
                    format!("{}::{}({})", type_name, name, args)
                })
            })
            .collect()
    }

    /// Helper to complete instance methods (e.g., "var.method")
    fn complete_instance_methods(
        &self,
        prefix: &str,
        storage: &Arc<Mutex<Storage>>,
    ) -> Vec<String> {
        // Split the prefix on the first '.' delimiter.
        let (var_name, method_prefix) = match prefix.split_once('.') {
            Some((v, m)) => (v, m),
            None => return Vec::new(),
        };

        // Look up the variable in storage, returning early if not found.
        let value = match storage.lock().unwrap().get(var_name) {
            Ok(val) => val,
            Err(_) => return Vec::new(),
        };

        let type_name = value.to_string();

        // Look up the struct information using the type name.
        let registry = self.registry.lock().unwrap();
        let struct_info = match registry.structs.get(type_name.as_str()) {
            Some(info) => info,
            None => return Vec::new(),
        };

        // Iterate over instance methods, filter by the user's input, and
        // build completions using a chained iterator.
        struct_info
            .instance_methods
            .iter()
            .filter(|(name, _)| name.starts_with(method_prefix))
            .flat_map(|(name, overloads)| {
                overloads.iter().map(move |method| {
                    // Build the argument string (e.g., "rows:i64, cols:i64").
                    let args = method
                        .args
                        .iter()
                        .map(|arg| format!("{}:{}", arg.name, crate::value::label(arg.type_name)))
                        .collect::<Vec<_>>()
                        .join(", ");
                    // Combine back into a completion string like "var.method(args)".
                    format!("{}.{}({})", var_name, name, args)
                })
            })
            .collect()
    }

    /// Helper to complete functions based on a prefix
    fn complete_functions(&self, prefix: &str) -> Vec<String> {
        // Acquire the registry lock
        let registry = self.registry.lock().unwrap();

        // Iterate over all functions in the registry
        registry
            .functions
            .iter()
            // Filter functions that match the prefix
            .filter(|(name, _)| name.starts_with(prefix))
            // For each matching function, expand its overloads
            .flat_map(|(name, overloads)| {
                // Create completions for each overload
                overloads.iter().map(move |function| {
                    // Format the function arguments
                    let args = function
                        .args
                        .iter()
                        .map(|arg| format!("{}:{}", arg.name, arg.type_name))
                        .collect::<Vec<_>>()
                        .join(", ");

                    // Build the complete function signature
                    format!("{}({})", name, args)
                })
            })
            .collect()
    }
}

impl Completer for FunctionCompleter {
    type Candidate = String; // Use `String` to allow returning dynamic completions

    fn complete(
        &self,
        line: &str,
        pos: usize,
        _ctx: &Context<'_>,
    ) -> Result<(usize, Vec<Self::Candidate>), ReadlineError> {
        // Find the starting index for the current word being completed
        let start = line[..pos].rfind(' ').map_or(0, |n| n + 1);
        let prefix = &line[start..pos];

        let mut matches = Vec::new();

        // Check if it's an instance method (e.g., "var.method")
        if prefix.contains(".") {
            matches.extend(self.complete_instance_methods(prefix, &self.storage));
        }

        // Check if it's a struct method (e.g., "Struct::method")
        if prefix.contains("::") {
            matches.extend(
                self.complete_struct_methods(prefix)
                    .into_iter()
                    .map(|method| {
                        format!("{}{}", prefix, method.strip_prefix(prefix).unwrap_or(""))
                    }),
            );
        }

        // Check if it's a function
        matches.extend(
            self.complete_functions(prefix)
                .into_iter()
                .map(|s| format!("{}", s)),
        );

        // Check if it's a stored variable
        matches.extend(
            self.complete_vars(prefix)
                .into_iter()
                .map(|var| format!("{var}")),
        );

        // Check if it's a struct type before :: is written
        matches.extend(
            self.complete_structs(prefix)
                .into_iter()
                .map(|s| format!("{}::", s)),
        );

        // Return the starting index and the matches
        Ok((start, matches))
    }
}
