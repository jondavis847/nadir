use crate::{ui::select_menu::SelectMenu, Message};
use geometry::Geometry;
use iced::{
    widget::{shader, Row},
    Element, Length, Point,
};
use multibody::result::MultibodyResult;
use polars::datatypes::AnyValue;

pub mod animator;
use animator::Animator;
pub mod scene;
use scene::{geometries::cuboid::Cuboid, Scene};
//use plot_canvas::PlotCanvas;

#[derive(Debug)]
pub struct AnimationTab {
    animator: Animator,
    pub sim_menu: SelectMenu,
    pub result: Option<MultibodyResult>,
    pub scene: Scene,
    is_pressed: bool,
}

impl Default for AnimationTab {
    fn default() -> Self {
        Self {
            animator: Animator::default(),
            sim_menu: SelectMenu::new(Length::FillPortion(1), false),
            result: None,
            scene: Scene::new(),
            is_pressed: false,
        }
    }
}

impl AnimationTab {
    pub fn animate(&mut self, instant: iced::time::Instant) {
        self.animator.update(instant);
        //dbg!(&self.animator);
        if let Some(result) = &self.result {
            for cuboid in &mut self.scene.cuboids {
                let (attitude, position) = result
                    .get_body_state_at_time_interp(&cuboid.name, self.animator.current_time as f64);
                cuboid.position =
                    glam::vec3(position.e1 as f32, position.e2 as f32, position.e3 as f32);
                cuboid.rotation = glam::quat(
                    attitude.x as f32,
                    attitude.y as f32,
                    attitude.z as f32,
                    attitude.s as f32,
                );
            }
        }
    }

    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let sim_menu = self
            .sim_menu
            .content(|string| Message::AnimationSimSelected(string));

        let shader_program = shader(&self.scene)
            .width(Length::FillPortion(10))
            .height(Length::Fill);

        Row::new()
            .push(sim_menu)
            .push(shader_program)
            .height(Length::FillPortion(15))
            .width(Length::Fill)
            .into()
    }

    pub fn left_button_pressed(&self, _position: Point) {
        //Nothing for now
    }

    pub fn sim_selected(&mut self, result: &MultibodyResult) {
        //TODO: This clone is probably really bad. We should not have to do this but I don't want to deal with lifetimes yet.
        // maybe do RC<RefCell<>> just for this?
        self.result = Some(result.clone());

        let mut cuboids = Vec::<Cuboid>::new();

        let sys = &result.system;
        for i in 0..sys.bodies.len() {
            let body = &sys.bodies[i];
            let q = body.state.attitude_base;
            let r = body.state.position_base;
            let rotation = glam::Quat::from_xyzw(q.x as f32, q.y as f32, q.z as f32, q.s as f32);
            let position = glam::vec3(r.e1 as f32, r.e2 as f32, r.e3 as f32);
            let body_name = &sys.body_names[i];

            if let Some(geometry) = body.geometry {
                match geometry {
                    Geometry::Cuboid(cuboid) => {
                        let cuboid = Cuboid::new(
                            body_name.clone(),
                            cuboid.length,
                            cuboid.width,
                            cuboid.height,
                            rotation,
                            position,
                        );
                        cuboids.push(cuboid);
                    }
                }
            }
        }
        self.scene.cuboids = cuboids;
        let t = &result.sim_time;

        let start_time = t[0] as f32;
        let end_time = t[t.len() - 1] as f32;

        let animator = Animator::new(start_time, end_time);
        self.animator = animator;
        self.animator.start();
    }
}
