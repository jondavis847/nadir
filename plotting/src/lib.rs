use core::f64;
use iced::widget::canvas::Frame;
use iced::{Point, Rectangle};
use inquire::Select;
use multibody::result::MultibodyResult;
use ron::de::from_reader;
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::Read;
use std::path::{Path, PathBuf};

mod application;
mod canvas;

#[derive(Debug)]
pub struct SeriesMap {
    map: HashMap<u32, Series>,
    next_id: u32,
    xmax: f32,
    xmin: f32,
    ymax: f32,
    ymin: f32,
}

impl Default for SeriesMap {
    fn default() -> Self {
        Self {
            map: HashMap::new(),
            next_id: 0,
            xmax: -f32::INFINITY,
            xmin: f32::INFINITY,
            ymax: -f32::INFINITY,
            ymin: f32::INFINITY,
        }
    }
}

impl SeriesMap {
    pub fn new() -> Self {
        SeriesMap::default()
    }

    pub fn get_relative_point(&self, point: &Point, frame: &Frame) -> Point {
        let x = (point.x - self.xmin) / (self.xmax - self.xmin);
        let y = (point.y - self.ymin) / (self.ymax - self.ymin);

        let center = frame.center();
        let width = frame.width();
        let height = frame.height();
        let left = center.x - width / 2.0;
        let top = center.y - height / 2.0;

        let x = left + x * width;
        let y = top + (height - y * height);

        Point::new(x, y)
    }
    pub fn insert(&mut self, series: Series) {
        if let Some((xmin, xmax, ymin, ymax)) = &series.find_boundaries() {
            if xmin < &self.xmin {
                self.xmin = *xmin;
            }
            if xmax > &self.xmax {
                self.xmax = *xmax;
            }
            if ymin < &self.ymin {
                self.ymin = *ymin;
            }
            if ymax > &self.ymax {
                self.ymax = *ymax;
            }

            self.map.insert(self.next_id, series);
            self.next_id += 1;
        } else {
            panic!("Could not find data boundaries. Check the data is not corrupted.")
        };
    }
}

#[derive(Debug)]
pub struct Series {
    pub result: String,
    pub component: String,
    pub state: String,
    pub points: Vec<Point>,
}

impl Series {
    fn new(result: String, component: String, state: String, x: Vec<f64>, y: Vec<f64>) -> Self {
        let points = x
            .into_iter()
            .zip(y)
            .map(|(x, y)| Point::new(x as f32, y as f32))
            .collect();

        Self {
            result,
            component,
            state,
            points,
        }
    }

    pub fn find_boundaries(&self) -> Option<(f32, f32, f32, f32)> {
        if self.points.is_empty() {
            return None;
        }

        let mut xmin = f32::INFINITY;
        let mut xmax = f32::NEG_INFINITY;
        let mut ymin = f32::INFINITY;
        let mut ymax = f32::NEG_INFINITY;

        for point in &self.points {
            if point.x.is_nan() || point.y.is_nan() {
                continue; // Skip points with NaN coordinates
            }
            if point.x < xmin {
                xmin = point.x;
            }
            if point.x > xmax {
                xmax = point.x;
            }
            if point.y < ymin {
                ymin = point.y;
            }
            if point.y > ymax {
                ymax = point.y;
            }
        }

        if xmin == f32::INFINITY
            || xmax == f32::NEG_INFINITY
            || ymin == f32::INFINITY
            || ymax == f32::NEG_INFINITY
        {
            None // All points were NaN
        } else {
            Some((xmin, xmax, ymin, ymax))
        }
    }
}

pub fn main(provided_path: Option<PathBuf>) {
    let mut id = 0 as u32;
    let mut series = SeriesMap::new();

    let path = match provided_path {
        Some(path) => path,
        None => match std::env::current_dir() {
            Ok(path) => path,
            Err(_) => panic!("Could not get current directory"),
        },
    };

    // Get the folder with the result.ron file
    let result_folder = if path.join("result.ron").is_file() {
        // already in a results folder with a results.ron file
        path
    } else if path.join("results").is_dir() {
        // in the root folder, where results is. select the result folder
        let result_folders = get_results(&path.join("results"));
        if result_folders.is_empty() {
            panic!("Could not find 'result.ron' in any subfolders or the current folder.")
        } else {
            if let Some(folder) = select_folder(result_folders) {
                folder
            } else {
                unreachable!("Selected folder did not exist. Code should not allow this.")
            }
        }
    } else {
        panic!("Could not find 'result.ron' in any subfolders or the current folder.")
    };

    let ron_path = result_folder.join("result.ron");
    let mut ron_file = File::open(ron_path).unwrap();
    let mut ron_content = String::new();
    ron_file.read_to_string(&mut ron_content).unwrap();
    let result: MultibodyResult = from_reader(ron_content.as_bytes()).unwrap();

    let mut components: Vec<String> = result.result.keys().cloned().collect();
    components.sort();
    let component_name = match Select::new("component:", components).prompt().ok() {
        Some(component) => component,
        None => unreachable!("Selected component did not exist. Code should not allow this."),
    };

    if let Some(component) = result.result.get(&component_name) {
        let mut states = component.keys();
        states.sort();
        let state_name = match Select::new("state:", states).prompt().ok() {
            Some(state) => state,
            None => unreachable!("Selected state did not exist. Code should not allow this."),
        };

        let x = result.sim_time;
        let y = component.get(&state_name).unwrap().clone();

        series.insert(Series::new(
            result.name.clone(),
            component_name,
            state_name,
            x,
            y,
        ));
        id += 1;
    } else {
        unreachable!("Component not found in the result map. Should not be possible.")
    }

    application::main(series);
}

/// Returns a `Vec<String>` containing the names of directories in the specified folder
/// that contain a file named `result.ron`.
///
/// # Arguments
/// * `result_folder` - A `Path` reference to the folder where the search will be conducted.
///
/// # Returns
/// * A vector of directory names (as `String`) that contain a `result.ron` file.
///
/// # Example
/// If the current folder structure is:
/// ```
/// /some_folder
/// ├── dir1
/// │   └── result.ron
/// ├── dir2
/// │   └── other_file.txt
/// └── dir3
///     └── result.ron
/// ```
/// The function will return `vec!["dir1", "dir3"]`.
fn get_results(result_folder: &Path) -> Vec<PathBuf> {
    // Vector to store the names of directories that contain the `result.ron` file
    let mut result_directories = Vec::new();

    // Attempt to read the contents of the specified directory
    if let Ok(entries) = fs::read_dir(result_folder) {
        // Iterate over each entry in the directory
        for entry in entries {
            // Ensure the entry was read successfully
            if let Ok(entry) = entry {
                // Get the full path of the current entry
                let path = entry.path();

                // Check if the current entry is a directory
                if path.is_dir() {
                    // Construct the path to `result.ron` within this directory
                    let result_file_path = path.join("result.ron");

                    // Check if the `result.ron` file exists and is a regular file
                    if result_file_path.is_file() {
                        // Add the full path of the directory to the results
                        result_directories.push(path.clone());
                    }
                }
            }
        }
    } else {
        // Print an error message if the directory could not be read
        eprintln!("Could not read directory: {:?}", result_folder);
    }

    // Return the vector of directories containing `result.ron`
    result_directories
}

fn select_folder(folders: Vec<PathBuf>) -> Option<PathBuf> {
    // Create a vector of folder names as strings for display in the menu
    let mut folder_names: Vec<String> = folders
        .iter()
        .filter_map(|path| {
            path.file_name()
                .and_then(|name| name.to_str().map(|s| s.to_string()))
        })
        .collect();
    folder_names.sort();

    // Use the `Select` prompt to present the folder names to the user
    let selection = Select::new("result folder:", folder_names).prompt();

    // Match the user's selection to return the corresponding `PathBuf`
    match selection {
        Ok(selected_name) => folders.into_iter().find(|path| {
            path.file_name()
                .and_then(|name| name.to_str())
                .map_or(false, |name| name == selected_name)
        }),
        Err(_) => None,
    }
}
