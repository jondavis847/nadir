use rustyline::completion::Completer;
use rustyline::error::ReadlineError;
use rustyline::highlight::{CmdKind, Highlighter, MatchingBracketHighlighter};
use rustyline::validate::MatchingBracketValidator;
use rustyline::Context;
use rustyline_derive::{Completer, Helper, Hinter, Validator};
use std::borrow::Cow::{self, Borrowed, Owned};
use std::cell::RefCell;
use std::rc::Rc;

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
    pub fn new(storage: Rc<RefCell<Storage>>) -> Self {
        NadirHelper {
            completer: FunctionCompleter::new(storage),
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
    storage: Rc<RefCell<Storage>>,
}

impl FunctionCompleter {
    const FUNCTIONS: [&'static str; 0] = [];
    const STRUCTS: [&'static str; 3] = ["Quaternion", "UnitQuaternion", "Vector"];
    const STRUCT_METHODS: [(&'static str, &'static str); 5] = [
        ("Quaternion", "new"),
        ("Quaternion", "rand"),
        ("UnitQuaternion", "new"),
        ("UnitQuaternion", "rand"),
        ("Vector", "rand"),
    ];
    const INSTANCE_METHODS: [(&'static str, &'static str); 2] =
        [("Quaternion", "inv"), ("UnitQuaternion", "inv")];

    pub fn new(storage: Rc<RefCell<Storage>>) -> Self {
        Self { storage }
    }

    /// Helper to complete function names
    fn complete_functions(prefix: &str) -> Vec<&'static str> {
        Self::FUNCTIONS
            .iter()
            .filter(|func| func.starts_with(prefix))
            .cloned()
            .collect()
    }

    /// Helper to complete struct names
    fn complete_structs(prefix: &str) -> Vec<&'static str> {
        Self::STRUCTS
            .iter()
            .filter(|s| s.starts_with(prefix))
            .cloned()
            .collect()
    }

    /// Helper to complete struct methods (e.g., "StructName.method")
    fn complete_methods(&self, prefix: &str) -> Vec<&'static str> {
        // Split prefix into struct and method if applicable
        if let Some((instance_name, method_prefix)) = prefix.split_once(".") {
            let storage = self.storage.borrow();
            match storage.get(instance_name) {
                Ok(value) => {
                    let type_name = value.to_string();
                    Self::INSTANCE_METHODS
                        .iter()
                        .filter(|(s, m)| s.starts_with(&type_name) && m.starts_with(method_prefix))
                        .map(|(_, method)| *method)
                        .collect()
                }
                Err(_) => {
                    vec![]
                }
            }
        } else if let Some((struct_name, method_prefix)) = prefix.split_once("::") {
            Self::STRUCT_METHODS
                .iter()
                .filter(|(s, m)| s.starts_with(struct_name) && m.starts_with(method_prefix))
                .map(|(_, method)| *method)
                .collect()
        } else {
            vec![]
        }
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
        if prefix.contains('.') {
            matches.extend(
                self.complete_methods(prefix).into_iter().map(|method| {
                    format!("{}{}", prefix, method.strip_prefix(prefix).unwrap_or(""))
                }),
            );
        }
        // Check if it's a struct method (e.g., "Struct::method")
        else if prefix.contains("::") {
            matches.extend(
                self.complete_methods(prefix).into_iter().map(|method| {
                    format!("{}{}", prefix, method.strip_prefix(prefix).unwrap_or(""))
                }),
            );
        } else {
            // Otherwise, check for function or struct completions
            matches.extend(
                Self::complete_functions(prefix)
                    .into_iter()
                    .map(|s| s.to_string()),
            );
            matches.extend(
                Self::complete_structs(prefix)
                    .into_iter()
                    .map(|s| s.to_string()),
            );
        }

        // Return the starting index and the matches
        Ok((start, matches))
    }
}
