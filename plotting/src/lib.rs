use core::f64;
use csv::ReaderBuilder;
use inquire::{Confirm, CustomType, MultiSelect, Select};
use std::collections::HashMap;
use std::fs::{self, File};
use std::path::{Path, PathBuf};

mod app;
mod axes;
mod axis;
mod canvas;
mod figure;
mod line;
mod series;
mod theme;
use series::{Series, SeriesMap};

use crate::figure::Figure;

#[derive(Debug, Default)]
pub struct Identifier {
    current: u32,
}

impl Identifier {
    fn next(&mut self) -> u32 {
        let value = self.current;
        self.current += 1;
        value
    }
}

#[derive(Debug, Default)]
pub struct PlotApp {
    identifier: Identifier,
    figures: HashMap<u32, Figure>,
}

impl PlotApp {
    pub fn new_figure(&mut self) -> Self {
        let id = self.identifier.next();
        let fig = Figure::new(id, window_size);
        self.figures.insert(id, fig);
    }
}

pub fn main(provided_path: Option<PathBuf>) {
    let mut series = Vec::new();

    // let root_path = provided_path
    //     .unwrap_or_else(|| std::env::current_dir().expect("could not get current directory"));

    // let mut first_pass = true;

    // // axes loop
    // loop {
    //     let mut map = if first_pass {
    //         first_pass = false;
    //         SeriesMap::new((0, 0))
    //     } else {
    //         let this_location = loop {
    //             let row: usize =
    //                 prompt_positive_integer("enter axes layout row (0 based indexing!)");
    //             let column: usize =
    //                 prompt_positive_integer("enter axes layout column (0 based indexing!)");

    //             let location = (row, column);

    //             if !series
    //                 .iter()
    //                 .any(|entry: &SeriesMap| entry.axes == location)
    //             {
    //                 break location; // Found a valid location
    //             } else {
    //                 println!("Error: axes location {:?} is taken", location);
    //             }
    //         };
    //         SeriesMap::new(this_location)
    //     };
    //     // series loop
    //     loop {
    //         let mut path = select_x_source(&root_path);
    //         while path.is_dir() {
    //             path = select_x_source(&path);
    //         }
    //         let x_name = select_x_data(&path);
    //         let x_data = extract_column(&path, &x_name);

    //         let mut path = select_y_source(&root_path);
    //         while path.is_dir() {
    //             path = select_y_source(&path);
    //         }
    //         let y_names = select_y_data(&path);
    //         for y_name in y_names {
    //             let y_data = extract_column(&path, &y_name);
    //             map.insert(Series::new(x_name.clone(), x_data.clone(), y_name, y_data));
    //         }

    //         // Ask the user if they want to add another series
    //         let add_another = Confirm::new("add another line?")
    //             .with_default(false)
    //             .prompt()
    //             .unwrap_or(false);
    //         if !add_another {
    //             break;
    //         }
    //     }
    //     series.push(map);
    //     // Ask the user if they want to add another axes
    //     let add_another = Confirm::new("add another axes?")
    //         .with_default(false)
    //         .prompt()
    //         .unwrap_or(false);
    //     if !add_another {
    //         break;
    //     }
    // }
    figure::main().expect("Application failed to run");
}

fn select_x_source(path: &PathBuf) -> PathBuf {
    let contents = get_contents(path);
    let choice = Select::new("select x-axis source", contents)
        .prompt()
        .expect("source selection failed");

    path.join(choice)
}

fn select_y_source(path: &PathBuf) -> PathBuf {
    let contents = get_contents(path);
    let choice = Select::new("select y-axis source", contents)
        .prompt()
        .expect("source selection failed");

    path.join(choice)
}

fn get_contents(path: &PathBuf) -> Vec<String> {
    let mut folders = Vec::new();
    let mut csvs = Vec::new();
    let mut f2s = Vec::new();
    // Iterate over the entries in the directory
    for entry in fs::read_dir(path).expect("could not read directory contents") {
        let entry = entry.unwrap();
        let path = entry.path();

        // Check if the entry is a directory
        if path.is_dir() {
            if let Some(folder_name) = path.file_name().and_then(|name| name.to_str()) {
                folders.push(format!(
                    "{}",
                    format!("{}{}", folder_name, std::path::MAIN_SEPARATOR)
                ));
            }
        }
        // Check if the entry is a file
        else if path.is_file() {
            if let Some(file_name) = path.file_name().and_then(|name| name.to_str()) {
                // Check for .csv extension
                if path.extension().and_then(|ext| ext.to_str()) == Some("csv") {
                    csvs.push(format!("{}", file_name));
                }
                // Check for .42 extension
                else if path.extension().and_then(|ext| ext.to_str()) == Some("42") {
                    f2s.push(format!("{}", file_name));
                }
            }
        }
    }
    let mut contents = Vec::new();
    folders.sort();
    csvs.sort();
    f2s.sort();
    contents.append(&mut folders);
    contents.append(&mut csvs);
    contents.append(&mut f2s);
    contents
}

fn get_csv_headers(path: &Path) -> Vec<String> {
    // Check if the file has a .csv extension
    if path.extension().and_then(|ext| ext.to_str()) != Some("csv") {
        panic!("The file {:?} does not have a .csv extension.", path);
    }

    // Attempt to open the file
    let file = File::open(path).unwrap();

    // Create a CSV reader with headers enabled
    let mut reader = ReaderBuilder::new().has_headers(true).from_reader(file);

    // Retrieve the headers
    reader
        .headers()
        .map_err(|_| "Could not read headers from the CSV file.")
        .unwrap()
        .iter()
        .map(|s| s.to_string())
        .collect::<Vec<String>>()
}

fn select_x_data(path: &Path) -> String {
    let headers = get_csv_headers(path);

    // Prompt the user to select a column
    Select::new("select x-axis data:", headers)
        .prompt()
        .map_err(|_| "Column selection failed.")
        .unwrap()
}

fn select_y_data(path: &Path) -> Vec<String> {
    let headers = get_csv_headers(path);

    // Prompt the user to select a column
    MultiSelect::new("select y-axis data:", headers)
        .prompt()
        .map_err(|_| "Column selection failed.")
        .unwrap()
}

fn extract_column(path: &Path, column_name: &str) -> Vec<f64> {
    // Attempt to open the CSV file
    let file = match File::open(path) {
        Ok(file) => file,
        Err(e) => {
            eprintln!("Error opening file {:?}: {}", path, e);
            return Vec::new();
        }
    };

    // Create a CSV reader with headers enabled
    let mut reader = ReaderBuilder::new().has_headers(true).from_reader(file);

    // Retrieve the headers to find the index of the desired column
    let headers = match reader.headers() {
        Ok(headers) => headers,
        Err(e) => {
            eprintln!("Error reading CSV headers: {}", e);
            return Vec::new();
        }
    };

    let column_index = match headers.iter().position(|header| header == column_name) {
        Some(index) => index,
        None => {
            eprintln!("Column '{}' not found in CSV headers.", column_name);
            return Vec::new();
        }
    };

    // Collect all values from the specified column and parse them as f64
    let mut column_values = Vec::new();
    for result in reader.records() {
        match result {
            Ok(record) => {
                if let Some(value) = record.get(column_index) {
                    match value.parse::<f64>() {
                        Ok(parsed_value) => column_values.push(parsed_value),
                        Err(e) => {
                            eprintln!("Error parsing value '{}' as f64: {}", value, e);
                            return Vec::new();
                        }
                    }
                }
            }
            Err(e) => {
                eprintln!("Error reading CSV record: {}", e);
                return Vec::new();
            }
        }
    }

    column_values
}

fn prompt_positive_integer(prompt_message: &str) -> usize {
    loop {
        match CustomType::new(prompt_message)
            .with_error_message("please enter a positive integer (0 based indexing!)")
            .prompt()
        {
            Ok(value) => break value,
            Err(err) => println!("Error: {:?}", err),
        }
    }
}
