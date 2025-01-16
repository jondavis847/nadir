mod animator;
mod scene;

use crate::Message;
use animator::Animator;
use celestial::CelestialBodies;
use glam::{DQuat, DVec3};
use iced::{
    alignment, keyboard,
    mouse::ScrollDelta,
    widget::{container, horizontal_space, shader, slider, text, Column, Row, Stack},
    window, Color, Element, Length, Point, Subscription, Task, Theme, Vector,
};
use nadir_3d::mesh::Mesh;
use scene::Scene;

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
pub struct AnimationGui {
    state: AnimationState,
    result: AnimationResult,
}

impl AnimationGui {
    pub fn new(result: AnimationResult) -> (Self, Task<Message>) {
        let mut state = AnimationState::default();
        state.initialize(&result);
        (
            Self {
                state,
                result: result,
            },
            Task::done(Message::Loaded),
        )
    }

    pub fn update(&mut self, message: Message) {
        let state = &mut self.state;

        match message {
            Message::AnimationTick(instant) => {
                state.animate(&self.result, instant);
            }
            Message::CameraFovChanged(value) => state.camera_fov_changed(value),
            Message::CameraRotation(delta) => state.camera_rotated(delta),
            Message::ChannelDataReceived => {}
            Message::EscapePressed => state.escape_pressed(),
            //Message::LeftButtonPressed(cursor) => state.left_button_pressed(cursor),
            //Message::LeftButtonReleased(cursor) => state.left_button_released(cursor),
            Message::Loaded => {
                state.loaded = true;
            }
            Message::PlaybackSpeedChanged(value) => state.playback_speed_changed(value),
            Message::MiddleButtonPressed(cursor) => state.middle_button_pressed(cursor),
            Message::RightButtonPressed(cursor) => state.right_button_pressed(cursor),
            Message::RightButtonReleased(cursor) => state.right_button_released(cursor),
            Message::WheelScrolled(delta) => state.wheel_scrolled(delta),
            Message::WindowResized(_size) => {}
        }
    }

    pub fn view(&self) -> Element<Message, Theme> {
        let surface = Column::new().width(Length::Fill).height(Length::Fill);

        let surface = match self.state.loaded {
            false => surface.push(
                container(
                    text("Loading...")
                        .align_x(alignment::Horizontal::Center)
                        .size(30),
                )
                .width(Length::Fill)
                .height(Length::Fill)
                .center(Length::Fill),
            ),
            true => surface.push(self.state.content()),
        };

        surface.into()
    }

    pub fn subscription(&self) -> Subscription<Message> {
        Subscription::batch(vec![
            window::frames().map(Message::AnimationTick),
            iced::event::listen_with(|event, _, _| match event {
                iced::Event::Window(window_event) => match window_event {
                    window::Event::Resized(size) => Some(Message::WindowResized(size)),
                    _ => None,
                },
                iced::Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => match key {
                    keyboard::Key::Named(keyboard::key::Named::Escape) => {
                        Some(Message::EscapePressed)
                    }
                    keyboard::Key::Named(keyboard::key::Named::Delete) => None,
                    //keyboard::Key::Named(keyboard::key::Named::Tab) => Some(Message::TabPressed),
                    _ => None,
                },
                _ => None,
            }),
        ])
    }

    pub fn theme(&self) -> Theme {
        Theme::Dark
    }
}

#[derive(Debug)]
pub struct AnimationState {
    animator: Animator,
    pub loaded: bool,
    //mouse: MouseProcessor,
    scene: Scene,
    pub show_menu: bool,
}

impl Default for AnimationState {
    fn default() -> Self {
        Self {
            animator: Animator::default(),
            loaded: false,
            scene: Scene::default(),
            show_menu: false,
        }
    }
}

impl AnimationState {
    pub fn animate(&mut self, result: &AnimationResult, instant: iced::time::Instant) {
        self.animator.update(instant);
        let t = self.animator.current_time;
        let time = &result.sim_time;
        result
            .meshes
            .iter()
            .zip(&mut self.scene.body_meshes)
            .for_each(|(result, mesh)| {
                let (q, r) = result.get_state_at_time_interp(t, time);
                mesh.update(r, q);
            });

        result.celestial_meshes.iter().for_each(|result| {
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

            let playback_speed_slider = slider(
                0.0..=100.0,
                self.animator.speed,
                Message::PlaybackSpeedChanged,
            );

            let camera_fov_text =
                text(format!("Camera FOV: {}", self.scene.camera.fov_y)).color(Color::WHITE);

            let camera_fov_slider = slider(
                1.0..=179.0,
                self.scene.camera.fov_y,
                Message::CameraFovChanged,
            );

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

        stack.into()
    }

    pub fn escape_pressed(&mut self) {
        self.show_menu = !self.show_menu;
    }

    pub fn middle_button_pressed(&self, _cursor: Point) {}

    pub fn playback_speed_changed(&mut self, value: f64) {
        self.animator.speed = value;
    }

    pub fn right_button_pressed(&self, _cursor: Point) {}

    pub fn right_button_released(&self, _cursor: Point) {}

    pub fn initialize(&mut self, result: &AnimationResult) {
        // initialize the multibody bodies
        for result in &result.meshes {
            let mut mesh = result.mesh.clone();
            mesh.update(result.position[0], result.attitude[0]);
            self.scene.body_meshes.push(mesh);
        }

        if !result.celestial_meshes.is_empty() {
            for result in &result.celestial_meshes {
                self.scene.celestial.add_body(result.body);
                self.scene.celestial.update_body(
                    result.body,
                    result.position[0],
                    result.attitude[0],
                );
            }
            self.scene.set_celestial();
        }

        let t = &result.sim_time;

        let start_time = t[0];
        let end_time = t[t.len() - 1];

        let animator = Animator::new(start_time, end_time);

        self.animator = animator;
        self.animator.start();
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
