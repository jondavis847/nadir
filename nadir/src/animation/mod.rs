mod animator;
mod celestial_animation;
mod scene;

use std::{
    path::{Path, PathBuf},
    time::Instant,
};

use animator::{Animator, AnimatorStatus};
use celestial::CelestialBodies;
use csv::ReaderBuilder;
use glam::{DQuat, DVec3, dquat, dvec3};
use iced::{
    Color, Element, Length, Point, Rectangle, Size, Theme, Vector,
    mouse::ScrollDelta,
    widget::{
        Column, Row, Stack, button, container, horizontal_space, shader, slider, text,
        vertical_space,
    },
    window::Id,
};
use nadir_3d::mesh::Mesh;
use ron::from_str;
use scene::Scene;

use thiserror::Error;

use crate::window_manager::Message;

#[derive(Debug, Error)]
pub enum AnimationErrors {
    #[error("{0}")]
    CsvErrors(#[from] csv::Error),
    #[error("no meshes to render in this result")]
    NothingToRender,
}

#[derive(Debug, Default)]
pub struct AnimationResult {
    pub sim_time: Vec<f64>,
    pub meshes: Vec<MeshResult>,
    pub celestial_meshes: Vec<CelestialResult>,
}

#[derive(Debug)]
pub struct MeshResult {
    pub mesh: Mesh,
    pub position: Vec<DVec3>,
    pub attitude: Vec<DQuat>,
}
impl MeshResult {
    fn get_state_at_time_interp(&self, t: f64, time: &Vec<f64>) -> (DQuat, DVec3) {
        get_state_at_time_interp(t, &time, &self.attitude, &self.position)
    }
}

#[derive(Debug)]
pub struct CelestialResult {
    pub body: CelestialBodies,
    pub position: Vec<DVec3>,
    pub attitude: Vec<DQuat>,
}

impl CelestialResult {
    fn get_state_at_time_interp(&self, t: f64, time: &Vec<f64>) -> (DQuat, DVec3) {
        get_state_at_time_interp(t, &time, &self.attitude, &self.position)
    }
}

#[derive(Debug)]
pub struct AnimationProgram {
    window_id: Option<Id>,
    result: AnimationResult,
    animator: Animator,
    scene: Scene,
    show_menu: bool,
    show_progress_bar: bool,
    progress_bounds: Rectangle,
}

impl AnimationProgram {
    pub fn camera_fov_changed(&mut self, value: f32) {
        self.scene.camera.set_fov(value);
    }

    pub fn camera_rotated(&mut self, mouse_delta: Vector) {
        self.scene
            .camera
            .update_position_from_mouse_delta(mouse_delta);
    }

    pub fn content(&self) -> Element<Message, Theme> {
        // we use a stack to put content on top of the animation
        // create the stack and add the shader program
        let mut stack =
            Stack::new().push(shader(&self.scene).width(Length::Fill).height(Length::Fill));

        // add the menu content if we should be
        if self.show_menu {
            let playback_speed_text =
                text(format!("Playback Speed: {}", self.animator.speed)).color(Color::WHITE);

            let playback_speed_slider = slider(0.0..=100.0, self.animator.speed, |speed| {
                Message::PlaybackSpeedChanged(self.window_id.unwrap(), speed)
            });

            let camera_fov_text =
                text(format!("Camera FOV: {}", self.scene.camera.fov_y)).color(Color::WHITE);

            let camera_fov_slider = slider(1.0..=179.0, self.scene.camera.fov_y, |fov| {
                Message::CameraFovChanged(self.window_id.unwrap(), fov)
            });

            let menu_column = Column::new()
                .push(playback_speed_text)
                .push(playback_speed_slider)
                .push(camera_fov_text)
                .push(camera_fov_slider);

            let menu = container(menu_column)
                .center_x(Length::FillPortion(1))
                .center_y(Length::Fill);
            let menu_row = Row::new()
                .push(horizontal_space().width(Length::FillPortion(1)))
                .push(menu)
                .push(horizontal_space().width(Length::FillPortion(1)));

            stack = stack.push(menu_row);
        }

        if self.show_progress_bar {
            let progress_slider = slider(
                self.animator.start_time..=self.animator.end_time,
                self.animator.current_time,
                |current_time| Message::CurrentTimeChanged(self.window_id.unwrap(), current_time),
            )
            .width(Length::FillPortion(20));

            let play_or_pause = match self.animator.status {
                AnimatorStatus::Paused => "||",
                AnimatorStatus::Playing => ">",
            };
            let play_or_pause = text(play_or_pause)
                .width(Length::Fill)
                .height(Length::Fill)
                .center();

            let play_button = button(play_or_pause)
                .on_press(Message::TogglePlayPause)
                .width(Length::FillPortion(2));

            let progress_row = Row::new()
                .push(horizontal_space().width(Length::FillPortion(3)))
                .push(play_button)
                .push(horizontal_space().width(Length::FillPortion(1)))
                .push(progress_slider)
                .push(horizontal_space().width(Length::FillPortion(3)));

            let progress_margin = vertical_space().height(Length::FillPortion(9));

            let progress_column = Column::new()
                .push(progress_margin)
                .push(progress_row)
                .height(Length::Fill)
                .width(Length::Fill);

            stack = stack.push(progress_column);
        }

        stack.into()
    }

    pub fn current_time_changed(&mut self, time: f64) {
        self.animator.current_time = time;
    }

    pub fn cursor_moved(&mut self, point: Point) {
        if self.progress_bounds.contains(point) {
            self.show_progress_bar = true;
        } else {
            self.show_progress_bar = false;
        }
    }
    pub fn escape_pressed(&mut self) {
        self.show_menu = !self.show_menu;
    }

    pub fn new(result_path: PathBuf) -> Result<Self, AnimationErrors> {
        let mut reader = ReaderBuilder::new()
            .has_headers(true) // Assuming the file has headers
            .from_path(&result_path.join("sim_time.csv"))?;
        let mut sim_time = Vec::new();
        for result in reader.records() {
            let record = result.unwrap();
            let t = record.get(0).unwrap().parse::<f64>().unwrap();
            sim_time.push(t);
        }

        let bodies_path = result_path.join("bodies");
        let mesh_results = get_mesh_result(&bodies_path);

        let celestial_path = result_path.join("celestial");
        let celestial_results = if celestial_path.is_dir() {
            get_celestial_result(&celestial_path)
        } else {
            Vec::new()
        };

        if mesh_results.is_empty() && celestial_results.is_empty() {
            return Err(AnimationErrors::NothingToRender);
        }

        let animation_result = AnimationResult {
            sim_time,
            meshes: mesh_results,
            celestial_meshes: celestial_results,
        };
        let mut scene = Scene::new();

        // initialize the multibody bodies
        for result in &animation_result.meshes {
            let mut mesh = result.mesh.clone();
            mesh.update(result.position[0], result.attitude[0]);
            scene.body_meshes.push(mesh);
        }

        if !animation_result.celestial_meshes.is_empty() {
            for result in &animation_result.celestial_meshes {
                scene.celestial.add_body(result.body);
                scene
                    .celestial
                    .update_body(result.body, result.position[0], result.attitude[0]);
            }
            scene.set_celestial();
        }

        let t = &animation_result.sim_time;

        let start_time = t[0];
        let end_time = t[t.len() - 1];

        let animator = Animator::new(start_time, end_time);

        let progress_bounds = Rectangle::new(Point::new(0.0, 600.0), Size::new(1280.0, 120.0));

        Ok(Self {
            window_id: None,
            animator,
            scene,
            show_menu: false,
            result: animation_result,
            show_progress_bar: false,
            progress_bounds,
        })
    }

    pub fn playback_speed_changed(&mut self, value: f64) {
        self.animator.speed = value;
    }

    pub fn set_window_id(&mut self, id: Id) {
        self.window_id = Some(id);
        self.scene.set_window_id(id);
    }

    pub fn tick(&mut self, instant: &Instant) {
        self.animator.update(instant);
        let t = self.animator.current_time;
        let time = &self.result.sim_time;
        self.result
            .meshes
            .iter()
            .zip(&mut self.scene.body_meshes)
            .for_each(|(result, mesh)| {
                let (q, r) = result.get_state_at_time_interp(t, time);
                mesh.update(r, q);
            });

        self.result.celestial_meshes.iter().for_each(|result| {
            let (q, r) = result.get_state_at_time_interp(t, time);
            // celestial position is in km, convert to m;
            self.scene.celestial.update_body(result.body, 1e3 * r, q);
        });

        // adjust mesh positions so target is at origin and all other meshes are relative to it
        if let Some(index) = self.scene.world_target {
            let camera_target = self.scene.body_meshes[index].state.position;
            for mesh in &mut self.scene.body_meshes {
                mesh.set_position_from_target(camera_target);
            }
            for (_, mesh) in &mut self.scene.celestial.meshes {
                mesh.set_position_from_target(camera_target);
            }
        }
    }

    pub fn toggle_play_pause(&mut self) {
        match self.animator.status {
            AnimatorStatus::Paused => self.animator.status = AnimatorStatus::Playing,
            AnimatorStatus::Playing => self.animator.status = AnimatorStatus::Paused,
        }
    }

    pub fn wheel_scrolled(&mut self, delta: ScrollDelta) {
        self.scene.camera.update_position_from_scroll_delta(delta);
    }
}

fn get_state_at_time_interp(
    t: f64,
    time: &Vec<f64>,
    q: &Vec<DQuat>,
    r: &Vec<DVec3>,
) -> (DQuat, DVec3) {
    match time.binary_search_by(|v| v.partial_cmp(&t).unwrap()) {
        Ok(i) => {
            // The target is exactly at index i
            (q[i], r[i])
        }
        Err(i) => {
            if i == 0 {
                // The target is smaller than the first element
                (q[i], r[i])
            } else if i == time.len() {
                // The target is greater than the last element
                (q[i], r[i])
            } else {
                // The target is between elements at i - 1 and i
                let t_prev = time[i - 1];
                let t_next = time[i];
                let s = (t - t_prev) / (t_next - t_prev); // between 0-1

                let interp_position = r[i - 1].lerp(r[i], s);
                let interp_attitude = q[i - 1].slerp(q[i], s);

                (interp_attitude, interp_position)
            }
        }
    }
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
