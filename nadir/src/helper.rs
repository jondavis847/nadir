use rustyline::Context;
use rustyline::completion::{Completer, FilenameCompleter};
use rustyline::error::ReadlineError;
use rustyline::highlight::{CmdKind, Highlighter, MatchingBracketHighlighter};
use rustyline::validate::MatchingBracketValidator;
use rustyline_derive::{Completer, Helper, Hinter, Validator};
use std::borrow::Cow::{self, Borrowed, Owned};
use std::path::PathBuf;
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
    pub fn new(
        registry: Arc<Mutex<Registry>>,
        storage: Arc<Mutex<Storage>>,
        current_dir: Arc<Mutex<PathBuf>>,
    ) -> Self {
        NadirHelper {
            completer: FunctionCompleter::new(registry, storage, current_dir),
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
    current_dir: Arc<Mutex<PathBuf>>,
    filename_completer: FilenameCompleter,
}

impl FunctionCompleter {
    pub fn new(
        registry: Arc<Mutex<Registry>>,
        storage: Arc<Mutex<Storage>>,
        current_dir: Arc<Mutex<PathBuf>>,
    ) -> Self {
        Self {
            registry,
            storage,
            current_dir,
            filename_completer: FilenameCompleter::new(),
        }
    }

    // /// Helper to complete function names
    // fn complete_enums(prefix: &str) -> Vec<&'static str> {
    //     Self::ENUMS
    //         .iter()
    //         .filter(|func| func.starts_with(prefix))
    //         .cloned()
    //         .collect()
    // }

    // Add this method
    fn complete_path(
        &self,
        line: &str,
        pos: usize,
        ctx: &Context<'_>,
    ) -> Result<(usize, Vec<String>), ReadlineError> {
        let (offset, completions) = self.filename_completer.complete(line, pos, ctx)?;
        let string_completions = completions.into_iter().map(|c| c.replacement).collect();
        Ok((offset, string_completions))
    }

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
    type Candidate = String;

    fn complete(
        &self,
        line: &str,
        pos: usize,
        ctx: &Context<'_>,
    ) -> Result<(usize, Vec<Self::Candidate>), ReadlineError> {
        // Check if we need path completion
        if line.starts_with("cd ") || line.starts_with("ls ") {
            // Find where the path argument begins
            let cmd_end = line.find(' ').unwrap_or(line.len());
            let path_start = if cmd_end < line.len() {
                cmd_end + 1
            } else {
                cmd_end
            };

            // Extract the path part being typed
            let path_part = &line[path_start..pos];

            // Get the current directory
            let current = self.current_dir.lock().unwrap().clone();

            // Find paths that match
            let mut matches = Vec::new();

            // Create the directory to search in
            let search_dir = if path_part.contains('/') || path_part.contains('\\') {
                // If there's a separator in the path, we need to resolve relative to current dir
                let mut full_path = current.clone();

                // Handle special paths
                if path_part.starts_with("~/") || path_part.starts_with("~\\") {
                    if let Some(home) = dirs::home_dir() {
                        full_path = home;
                        // Remove the ~ from path_part for matching
                        let path_suffix = &path_part[2..];
                        if !path_suffix.is_empty() {
                            full_path.push(path_suffix);
                        }
                    }
                } else if path_part.starts_with('/') || path_part.starts_with('\\') {
                    // Absolute path
                    #[cfg(unix)]
                    {
                        full_path = PathBuf::from("/");
                        if path_part.len() > 1 {
                            full_path.push(&path_part[1..]);
                        }
                    }
                    #[cfg(windows)]
                    {
                        // On Windows, need to handle drive letters carefully
                        if let Some(root) = current.components().next() {
                            full_path = PathBuf::from(root.as_os_str());
                            if path_part.len() > 1 {
                                full_path.push(&path_part[1..]);
                            }
                        }
                    }
                } else {
                    // Relative path with directory component
                    let last_sep = path_part
                        .rfind('/')
                        .or_else(|| path_part.rfind('\\'))
                        .unwrap();
                    let dir_part = &path_part[..=last_sep];
                    full_path.push(dir_part);
                }

                full_path
            } else {
                // No separators, just search in the current directory
                current
            };

            // Get the prefix to match against file names
            let name_prefix = if path_part.contains('/') || path_part.contains('\\') {
                path_part
                    .rfind('/')
                    .or_else(|| path_part.rfind('\\'))
                    .map(|i| &path_part[i + 1..])
                    .unwrap_or(path_part)
            } else {
                path_part
            };

            // Try to read the directory contents
            if let Ok(entries) = std::fs::read_dir(&search_dir) {
                for entry in entries {
                    if let Ok(entry) = entry {
                        if let Some(name) = entry.file_name().to_str() {
                            if name.starts_with(name_prefix) {
                                // Compute the completion text
                                let base_path =
                                    if path_part.contains('/') || path_part.contains('\\') {
                                        let last_sep = path_part
                                            .rfind('/')
                                            .or_else(|| path_part.rfind('\\'))
                                            .unwrap();
                                        path_part[..=last_sep].to_string()
                                    } else {
                                        String::new()
                                    };

                                // Build the completion
                                let mut completion = format!("{}{}", base_path, name);

                                // Add a trailing slash for directories
                                if entry.file_type().map(|ft| ft.is_dir()).unwrap_or(false) {
                                    use std::path::MAIN_SEPARATOR;
                                    completion.push(MAIN_SEPARATOR);
                                }

                                matches.push(completion);
                            }
                        }
                    }
                }
            }

            return Ok((path_start, matches));
        }

        // Find the starting index for the current word being completed
        let start = line[..pos].rfind(' ').map_or(0, |n| n + 1);
        let prefix = &line[start..pos];

        // Determine completion strategy based on prefix pattern
        let matches = if prefix.contains('.') {
            // Instance method completion (var.method)
            self.complete_instance_methods(prefix, &self.storage)
        } else if prefix.contains("::") {
            // Struct method completion (Struct::method)
            self.complete_struct_methods(prefix)
                .into_iter()
                .map(|method| {
                    let suffix = method.strip_prefix(prefix).unwrap_or("");
                    format!("{prefix}{suffix}")
                })
                .collect()
        } else {
            // Try multiple completion types and combine results
            let mut completions = Vec::new();

            // Functions
            completions.extend(self.complete_functions(prefix));

            // Variables
            completions.extend(self.complete_vars(prefix));

            // Struct types (adding :: suffix)
            completions.extend(
                self.complete_structs(prefix)
                    .into_iter()
                    .map(|s| format!("{}::", s)),
            );

            completions
        };

        Ok((start, matches))
    }
}
