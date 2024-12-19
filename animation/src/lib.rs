use aerospace::celestial_system::CelestialBodies;
use csv::ReaderBuilder;
use glam::{dquat, dvec3, DQuat, DVec3};
use nadir_3d::mesh::Mesh;
use ron::from_str;
use std::path::{Path, PathBuf};

mod animation;
mod celestial;
//mod mouse;

use animation::{AnimationGui, AnimationResult, CelestialResult, MeshResult};
use iced::{
    mouse::ScrollDelta,
    window::{self, icon},
    Point, Size, Vector,
};
use std::time::Instant;

#[derive(Debug)]
pub enum AnimationErrors {}

// Define the possible user interactions
#[derive(Debug, Clone)]
pub enum Message {
    AnimationTick(Instant),
    CameraRotation(Vector),
    CameraFovChanged(f32),
    ChannelDataReceived,
    EscapePressed,
    //LeftButtonPressed(Point),
    //LeftButtonReleased(Point),
    Loaded,
    PlaybackSpeedChanged(f64),
    MiddleButtonPressed(Point),
    RightButtonPressed(Point),
    RightButtonReleased(Point),
    WheelScrolled(ScrollDelta),
    WindowResized(Size),
}

pub fn main(result_path: Option<PathBuf>) -> iced::Result {    
    let pwd = match result_path {
        Some(path) => path,
        None => std::env::current_dir().unwrap(),
    };

    let mut reader = ReaderBuilder::new()
        .has_headers(true) // Assuming the file has headers
        .from_path(&pwd.join("sim_time.csv"))
        .unwrap();
    let mut sim_time = Vec::new();
    for result in reader.records() {
        let record = result.unwrap();
        let t = record.get(0).unwrap().parse::<f64>().unwrap();
        sim_time.push(t);
    }

    let bodies_path = pwd.join("bodies");
    let mesh_results = get_mesh_result(&bodies_path);

    let celestial_path = pwd.join("celestial");
    let celestial_results = if celestial_path.is_dir() {
        get_celestial_result(&celestial_path)
    }  else {
        Vec::new()
    };

    if mesh_results.is_empty() && celestial_results.is_empty() {
        panic!("no meshes found to animate")
    }

    let animation_result = AnimationResult {
        sim_time,
        meshes: mesh_results,
        celestial_meshes: celestial_results,
    };

    // load the icon
    const ICON: &[u8] = include_bytes!("../resources/icon.png");
    let icon_image = image::load_from_memory(ICON).expect("Failed to load icon");
    let icon_rgba = icon_image.to_rgba8();
    let (icon_width, icon_height) = icon_rgba.dimensions();
    let icon = icon::from_rgba(icon_rgba.into_vec(), icon_width, icon_height).unwrap();

    let mut window_settings = window::Settings::default();
    window_settings.size = Size::new(1280.0, 720.0);
    window_settings.icon = Some(icon);

    iced::application("NADIR", AnimationGui::update, AnimationGui::view)
        .antialiasing(true)
        .window(window_settings)
        .subscription(AnimationGui::subscription)
        .theme(AnimationGui::theme)
        .run_with(move || AnimationGui::new(animation_result))
}

fn get_mesh_result(bodies_path: &Path) -> Vec<MeshResult> {
    let mut results = Vec::new();

    // Read the directory
    for entry in std::fs::read_dir(bodies_path).unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();

        // Check if the file has a .csv extension
        if path.is_file() && path.extension().and_then(|ext| ext.to_str()) == Some("csv") {
            // Extract the file stem (name without extension)
            if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                // Construct the corresponding .mesh file path
                let mesh_file_path = bodies_path.join(format!("{}.mesh", stem));

                // Check if the .mesh file exists
                if mesh_file_path.exists() {
                    let mesh_str =
                        std::fs::read_to_string(mesh_file_path).expect("Failed to read .mesh file");
                    let mesh: Mesh = from_str(&mesh_str).expect("Failed to deserialize .mesh file");

                    let (attitude, position) = read_csv_result(&path);
                    results.push(MeshResult {
                        mesh,
                        position,
                        attitude,
                    })
                }
            }
        }
    }
    results
}

fn get_celestial_result(bodies_path: &Path) -> Vec<CelestialResult> {
    let mut results = Vec::new();

    // Read the directory
    for entry in std::fs::read_dir(bodies_path).unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();

        // Check if the file has a .csv extension
        if path.is_file() && path.extension().and_then(|ext| ext.to_str()) == Some("csv") {
            if let Some(stem) = path.file_stem().and_then(|s| s.to_str()) {
                if let Some(body) = CelestialBodies::from_str(stem) {
                    let (attitude, position) = read_csv_result(&path);
                    results.push(CelestialResult {
                        body,
                        attitude,
                        position,
                    })
                }
            }
        }
    }
    results
}

fn read_csv_result(path: &Path) -> (Vec<DQuat>, Vec<DVec3>) {
    // Open the CSV file
    let mut reader = ReaderBuilder::new()
        .has_headers(true) // Assuming the file has headers
        .from_path(path)
        .unwrap();
    let headers = reader.headers().unwrap();
    let irx = headers
        .iter()
        .position(|h| h == "position(base)[x]")
        .ok_or("Column 'position(base)[x]' not found")
        .unwrap();
    let iry = headers
        .iter()
        .position(|h| h == "position(base)[y]")
        .ok_or("Column 'position(base)[y]' not found")
        .unwrap();
    let irz = headers
        .iter()
        .position(|h| h == "position(base)[z]")
        .ok_or("Column 'position(base)[z]' not found")
        .unwrap();
    let iqx = headers
        .iter()
        .position(|h| h == "attitude(base)[x]")
        .ok_or("Column 'attitude(base)[x]' not found")
        .unwrap();
    let iqy = headers
        .iter()
        .position(|h| h == "attitude(base)[y]")
        .ok_or("Column 'attitude(base)[y]' not found")
        .unwrap();
    let iqz = headers
        .iter()
        .position(|h| h == "attitude(base)[z]")
        .ok_or("Column 'attitude(base)[z]' not found")
        .unwrap();
    let iqw = headers
        .iter()
        .position(|h| h == "attitude(base)[w]")
        .ok_or("Column 'attitude(base)[w]' not found")
        .unwrap();

    let mut position = Vec::new();
    let mut attitude = Vec::new();

    // Iterate through records
    for result in reader.records() {
        let record = result.unwrap();
        let rx = record.get(irx).unwrap().parse::<f64>().unwrap();
        let ry = record.get(iry).unwrap().parse::<f64>().unwrap();
        let rz = record.get(irz).unwrap().parse::<f64>().unwrap();
        let qx = record.get(iqx).unwrap().parse::<f64>().unwrap();
        let qy = record.get(iqy).unwrap().parse::<f64>().unwrap();
        let qz = record.get(iqz).unwrap().parse::<f64>().unwrap();
        let qw = record.get(iqw).unwrap().parse::<f64>().unwrap();

        position.push(dvec3(rx, ry, rz));
        attitude.push(dquat(qx, qy, qz, qw));
    }
    (attitude, position)
}
