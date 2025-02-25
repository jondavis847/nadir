use rustyline::completion::Completer;
use rustyline::error::ReadlineError;
use rustyline::highlight::{CmdKind, Highlighter, MatchingBracketHighlighter};
use rustyline::validate::MatchingBracketValidator;
use rustyline::Context;
use rustyline_derive::{Completer, Helper, Hinter, Validator};
use std::borrow::Cow::{self, Borrowed, Owned};
use std::cell::RefCell;
use std::rc::Rc;

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
    pub fn new(registry: Rc<RefCell<Registry>>, storage: Rc<RefCell<Storage>>) -> Self {
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
    registry: Rc<RefCell<Registry>>,
    storage: Rc<RefCell<Storage>>,
}

impl FunctionCompleter {
    pub fn new(registry: Rc<RefCell<Registry>>, storage: Rc<RefCell<Storage>>) -> Self {
        Self { registry, storage }
    }

    /// Helper to complete function names
    // fn complete_enums(prefix: &str) -> Vec<&'static str> {
    //     Self::ENUMS
    //         .iter()
    //         .filter(|func| func.starts_with(prefix))
    //         .cloned()
    //         .collect()
    // }

    /// Helper to complete function names
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
            .borrow()
            .get_structs()
            .iter()
            .filter(|s| s.starts_with(prefix))
            .cloned()
            .collect()
    }

    /// Helper to complete struct methods (e.g., "StructName::method")
    fn complete_methods(&self, prefix: &str) -> Vec<String> {
        // If the user typed something like "MyStruct::f"
        if let Some((type_name, method_prefix)) = prefix.split_once("::") {
            let methods = self
                .registry
                .borrow()
                .get_struct_methods(type_name)
                .unwrap_or_default();

            methods
                .iter()
                // Filter out methods that start with the user’s partial input (`method_prefix`)
                .filter(|method| method.starts_with(method_prefix))
                // Build a "TypeName::method" string for the user to complete with
                .map(|method| format!("{}::{}", type_name, method))
                .collect()
        } else {
            // If there's no "::" in the prefix, we're not looking up methods yet
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
            // Otherwise, check for function or struct or enum completions
            // matches.extend(
            //     Self::complete_functions(prefix)
            //         .into_iter()
            //         .map(|s| s.to_string()),
            // );
            matches.extend(
                self.complete_structs(prefix)
                    .into_iter()
                    .map(|s| format!("{}::", s)),
            );
            // matches.extend(
            //     Self::complete_enums(prefix)
            //         .into_iter()
            //         .map(|s| s.to_string()),
            // );
        }

        // Return the starting index and the matches
        Ok((start, matches))
    }
}
