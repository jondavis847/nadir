mod animator;
mod scene;

use crate::Message;
use animator::Animator;
use glam::DVec3;
use iced::{
    mouse::ScrollDelta,
    widget::{shader, Row},
    Command, Element, Length, Point, Theme, Vector,
};

use multibody::result::MultibodyResult;
use scene::Scene;

#[derive(Debug)]
pub struct AnimationGui {
    pub state: AnimationState,
    pub result: Option<MultibodyResult>,
}

impl AnimationGui {}

#[derive(Debug, Default)]
pub struct AnimationState {
    animator: Animator,
    pub loaded: bool,
    //mouse: MouseProcessor,
    scene: Scene,
}

impl AnimationState {
    pub fn animate(&mut self, result: &MultibodyResult, instant: iced::time::Instant) {
        self.animator.update(instant);

        for mesh in &mut self.scene.meshes {
            let (attitude, position) =
                result.get_body_state_at_time_interp(&mesh.name, self.animator.current_time as f64);
            let position = glam::dvec3(position[0], position[1], position[2]);
            let rotation = glam::dquat(attitude.x, attitude.y, attitude.z, attitude.s);
            mesh.update(position, rotation);
        }

        // adjust mesh positions so target is at origin and all other meshes are relative to it
        let camera_target = if let Some(index) = self.scene.world_target {
            let camera_target = self.scene.meshes[index].state.position;
            for mesh in &mut self.scene.meshes {
                mesh.set_position_from_target(camera_target);
            }
            camera_target
        } else {
            DVec3::ZERO
        };

        if let Some(earth) = &mut self.scene.earth {
            const ROTATION_RATE: f64 = 2.0 * std::f64::consts::PI / 86400.0;
            let rotation_axis = DVec3::Z;

            // Calculate the angle of rotation for this time step
            let angle = ROTATION_RATE * self.animator.dt;

            // Create a quaternion representing this incremental rotation
            let incremental_rotation = glam::DQuat::from_axis_angle(rotation_axis, angle);

            // Update the existing quaternion by applying the incremental rotation
            earth.0.state.rotation = incremental_rotation * (earth.0.state.rotation);

            // Adjust earth position based on camera target
            earth.0.state.position = DVec3::ZERO; // reset to 0.0 first or earth accumulates the error
            earth.0.set_position_from_target(camera_target);
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

        for i in 0..sys.bodies.len() {
            let body = &sys.bodies[i];
            let q = body.state.attitude_base;
            let r = body.state.position_base;
            let rotation = glam::DQuat::from_xyzw(q.x, q.y, q.z, q.s);
            let position = glam::dvec3(r[0], r[1], r[2]);
            if let Some(mesh) = &body.mesh {
                let mut mesh = mesh.clone();
                mesh.update(position, rotation);
                self.scene.meshes.push(mesh);
            }
        }

        if let Some(celestial) = &sys.base.celestial {
            if celestial.bodies.earth.is_some() {
                // set the base to be earth if it is in sys. for now this just animates an earth and moves the camera and light
                // otherwise defaults to close to the origin
                // must do this after shapres are set in scene or it wont know where to place the camera
                self.scene.set_earth(true);
            }
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
