mod animator;
mod scene;

use crate::Message;
use animator::Animator;
use iced::{
    alignment, keyboard,
    mouse::ScrollDelta,
    widget::{container, horizontal_space, shader, slider, text, Column, Row, Stack},
    window, Color, Element, Length, Point, Subscription, Task, Theme, Vector,
};

use multibody::result::{MultibodyResult, ResultEntry};
use scene::Scene;

#[derive(Debug)]
pub struct AnimationGui {
    pub state: AnimationState,
    pub result: Option<MultibodyResult>,
}

impl AnimationGui {
    pub fn new(result: MultibodyResult) -> (Self, Task<Message>) {
        let mut state = AnimationState::default();
        state.initialize(&result);
        (
            Self {
                state,
                result: Some(result),
            },
            Task::done(Message::Loaded),
        )
    }

    pub fn update(&mut self, message: Message) {
        let state = &mut self.state;

        match message {
            Message::AnimationTick(instant) => {
                if self.result.is_some() {
                    state.animate(self.result.as_ref().unwrap(), instant);
                }
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
    pub fn animate(&mut self, result: &MultibodyResult, instant: iced::time::Instant) {
        self.animator.update(instant);

        for mesh in &mut self.scene.body_meshes {
            let (attitude, position) =
                result.get_body_state_at_time_interp(&mesh.name, self.animator.current_time as f64);
            let position = glam::dvec3(position[0], position[1], position[2]);
            let rotation = glam::dquat(attitude.x, attitude.y, attitude.z, attitude.s);
            mesh.update(position, rotation);
        }

        for (body, mesh) in &mut self.scene.celestial.meshes {
            if let Some((attitude, position)) = result.get_celestial_state_at_time_interp(
                body.to_body(),
                self.animator.current_time as f64,
            ) {
                let position = glam::dvec3(position[0], position[1], position[2]) * 1e3; //celestial positions in km, convert to m
                let rotation = glam::dquat(attitude.x, attitude.y, attitude.z, attitude.s);
                mesh.update(position, rotation);
            }
        }

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

    pub fn initialize(&mut self, result: &MultibodyResult) {
        let sys = &result.system;

        // initialize the multibody bodies
        for i in 0..sys.bodies.len() {
            let body = &sys.bodies[i];
            let q = body.state.attitude_base;
            let r = body.state.position_base;
            let rotation = glam::DQuat::from_xyzw(q.x, q.y, q.z, q.s);
            let position = glam::dvec3(r[0], r[1], r[2]);
            if let Some(mesh) = &body.mesh {
                let mut mesh = mesh.clone();
                mesh.update(position, rotation);
                self.scene.body_meshes.push(mesh);
            }
        }

        // initialize the celestial bodies
        if let Some(celestial_entry) = result.result.get("celestial") {
            match celestial_entry {
                ResultEntry::Celestial(celestial) => {
                    for body in &celestial.bodies {
                        self.scene.celestial.add_body(body.body);

                        // get initial state
                        let position = body.position[0];
                        let orientation = body.orientation[0];

                        // convert to graphics types
                        //position of celestials is in km, convert to m for accurate graphics
                        let position = glam::dvec3(position[0], position[1], position[2]) * 1e3;
                        let orientation =
                            glam::dquat(orientation.x, orientation.y, orientation.z, orientation.s);

                        // update the mesh state and push to scene
                        self.scene
                            .celestial
                            .update_body(body.body, position, orientation);
                    }
                }
                _ => unreachable!("celestial should always be a CelestialResult"),
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
