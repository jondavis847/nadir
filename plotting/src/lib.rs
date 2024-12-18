use core::f64;
use csv::ReaderBuilder;
use inquire::{Confirm, Select};
use std::fs::{self, File};
use std::path::{Path, PathBuf};

mod application;
mod canvas;
mod series;
mod theme;
use series::{Series, SeriesMap};

pub fn main(provided_path: Option<PathBuf>) {
    let mut series = SeriesMap::new();

    let root_path = match provided_path {
        Some(p) => p,
        None => std::env::current_dir().expect("could not get current directory"),
    };

    loop {
        println!("select x data");
        let mut path = select_source(&root_path);
        while path.is_dir() {
            path = select_source(&path);
        }
        let x_name = prompt_csv_headers(&path).unwrap();
        let x_data = extract_column(&path, &x_name);

        println!("select y data:");
        let mut path = select_source(&root_path);
        while path.is_dir() {
            path = select_source(&path);
        }

        let y_name = prompt_csv_headers(&path).unwrap();
        let y_data = extract_column(&path, &y_name);

        series.insert(Series::new(x_name, x_data, y_name, y_data));

        // Ask the user if they want to add another series
        let add_another = Confirm::new("add another?").prompt().unwrap_or(false);
        if !add_another {
            break;
        }
    }

    application::main(series).expect("Application failed to run");
}

fn select_source(path: &PathBuf) -> PathBuf {
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
                folders.push(format!("{}\\", folder_name));
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

    let choice = Select::new("select source:", contents)
        .prompt()
        .expect("source selection failed");

    path.join(choice)
}

fn prompt_csv_headers(path: &Path) -> Result<String, Box<dyn std::error::Error>> {
    // Check if the path is a file
    if !path.is_file() {
        return Err(format!("The path {:?} is not a file.", path).into());
    }

    // Check if the file has a .csv extension
    if path.extension().and_then(|ext| ext.to_str()) != Some("csv") {
        return Err(format!("The file {:?} does not have a .csv extension.", path).into());
    }

    // Attempt to open the file
    let file = File::open(path)?;

    // Create a CSV reader with headers enabled
    let mut reader = ReaderBuilder::new().has_headers(true).from_reader(file);

    // Retrieve the headers
    let headers = reader
        .headers()
        .map_err(|_| "Could not read headers from the CSV file.")?
        .iter()
        .map(|s| s.to_string())
        .collect::<Vec<String>>();

    // Prompt the user to select a column
    let choice = Select::new("Select column:", headers)
        .prompt()
        .map_err(|_| "Column selection failed.")?;

    Ok(choice)
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
