mod animator;
mod scene;

use crate::Message;
use animator::Animator;
use iced::{
    mouse::ScrollDelta,
    widget::{shader, Row},
    Command, Element, Length, Point, Theme, Vector,
};

use multibody::result::{MultibodyResult, ResultEntry};
use scene::Scene;

#[derive(Debug)]
pub struct AnimationGui {
    pub state: AnimationState,
    pub result: Option<MultibodyResult>,
}

impl AnimationGui {}

#[derive(Debug)]
pub struct AnimationState {
    animator: Animator,
    pub loaded: bool,
    //mouse: MouseProcessor,
    scene: Scene,
}

impl Default for AnimationState {
    fn default() -> Self {
        Self {
            animator: Animator::default(),
            loaded: false,
            scene: Scene::default(),
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

    pub fn camera_rotated(&mut self, mouse_delta: Vector) -> Command<Message> {
        self.scene
            .camera
            .update_position_from_mouse_delta(mouse_delta);
        Command::none()
    }

    pub fn content(&self) -> Element<Message, Theme> {
        {
            let shader_program = shader(&self.scene)
                .width(Length::FillPortion(10))
                .height(Length::Fill);

            Row::new()
                .push(shader_program)
                .height(Length::FillPortion(15))
                .width(Length::Fill)
                .into()
        }
    }

    pub fn middle_button_pressed(&self, _cursor: Point) -> Command<Message> {
        Command::none()
    }

    pub fn right_button_pressed(&self, _cursor: Point) -> Command<Message> {
        Command::none()
    }

    pub fn right_button_released(&self, _cursor: Point) -> Command<Message> {
        Command::none()
    }

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

    pub fn wheel_scrolled(&mut self, delta: ScrollDelta) -> Command<Message> {
        self.scene.camera.update_position_from_scroll_delta(delta);
        Command::none()
    }
}
