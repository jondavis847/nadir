mod animator;
mod scene;

use crate::{mouse::MouseProcessor, Message};
use animator::Animator;
use geometry::Geometry;
use glam::{Quat, Vec3};
use iced::{
    mouse::ScrollDelta,
    widget::{shader, Row},
    Command, Element, Length, Point, Theme, Vector,
};
use multibody::result::MultibodyResult;
use scene::geometries::{cuboid::Cuboid, ellipsoid::Ellipsoid};
use scene::Scene;

#[derive(Debug, Clone)]
pub struct AnimationGui {
    pub state: AnimationState,
    pub result: Option<MultibodyResult>,
}

impl AnimationGui {}

#[derive(Debug, Default, Clone)]
pub struct AnimationState {
    animator: Animator,
    pub loaded: bool,
    mouse: MouseProcessor,
    scene: Scene,
}

impl AnimationState {
    pub fn animate(&mut self, result: &MultibodyResult, instant: iced::time::Instant) {
        self.animator.update(instant);
        for cuboid in &mut self.scene.cuboids {
            let (attitude, position) = result
                .get_body_state_at_time_interp(&cuboid.name, self.animator.current_time as f64);
            cuboid.position =
                glam::vec3(position[0] as f32, position[1] as f32, position[2] as f32);
            cuboid.rotation = glam::quat(
                attitude.x as f32,
                attitude.y as f32,
                attitude.z as f32,
                attitude.s as f32,
            );
        }

        for ellipsoid in &mut self.scene.ellipsoids {
            let (attitude, position) = result
                .get_body_state_at_time_interp(&ellipsoid.name, self.animator.current_time as f64);
            ellipsoid.position =
                glam::vec3(position[0] as f32, position[1] as f32, position[2] as f32);
                ellipsoid.rotation = glam::quat(
                attitude.x as f32,
                attitude.y as f32,
                attitude.z as f32,
                attitude.s as f32,
            );
        }
    }

    pub fn camera_rotated(&mut self, _delta: Vector) -> Command<Message> {
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

    pub fn update_from_result(&mut self, result: &MultibodyResult) {
        //TODO: This clone is probably really bad. We should not have to do this but I don't want to deal with lifetimes yet.
        // maybe do RC<RefCell<>> just for this?
        let mut cuboids = Vec::<Cuboid>::new();
        let mut ellipsoids = Vec::<Ellipsoid>::new();

        let sys = &result.system;
        for i in 0..sys.bodies.len() {
            let body = &sys.bodies[i];
            let q = body.state.attitude_base;
            let r = body.state.position_base;
            let rotation = glam::Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.s as f32);
            let position = glam::vec3(r[0] as f32, r[1] as f32, r[2] as f32);
            let body_name = &sys.body_names[i];

            if let Some(geometry) = body.geometry {
                match geometry {
                    Geometry::Cuboid(cuboid) => {
                        let cuboid = Cuboid::new(
                            body_name.clone(),
                            cuboid.x,
                            cuboid.y,
                            cuboid.z,
                            rotation,
                            position,
                            cuboid.color,
                        );
                        cuboids.push(cuboid);
                    }
                    Geometry::Ellipsoid(ellipsoid) => {
                        let ellipsoid = Ellipsoid::new(
                            body_name.clone(),
                            ellipsoid.radius_x,
                            ellipsoid.radius_y,
                            ellipsoid.radius_z,
                            rotation,
                            position,
                            ellipsoid.latitude_bands,
                            ellipsoid.longitude_bands,
                            ellipsoid.color,
                        );
                        ellipsoids.push(ellipsoid);
                    }
                }
            }
        }

        self.scene.cuboids = cuboids;
        self.scene.ellipsoids = ellipsoids;

        let t = &result.sim_time;

        let start_time = t[0] as f32;
        let end_time = t[t.len() - 1] as f32;

        let animator = Animator::new(start_time, end_time);

        self.animator = animator;
        self.animator.start();
    }

    pub fn wheel_scrolled(&mut self, _delta: ScrollDelta) -> Command<Message> {
        Command::none()
    }
}
