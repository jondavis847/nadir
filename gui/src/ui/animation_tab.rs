use crate::{ui::select_menu::SelectMenu, Message};
use iced::{
    widget::{shader, Row},
    Element, Length, Point,
};

use geometry::Geometry;
use multibody::result::MultibodyResult;

pub mod scene;
use scene::{geometry::cuboid::Cuboid, Scene};
//use plot_canvas::PlotCanvas;

#[derive(Debug)]
pub struct AnimationTab {
    pub sim_menu: SelectMenu,
    pub selected_sims: Vec<String>,
    pub scene: Scene,
    is_pressed: bool,
}

impl Default for AnimationTab {
    fn default() -> Self {
        Self {
            sim_menu: SelectMenu::new(Length::FillPortion(1), false),
            selected_sims: Vec::new(),
            scene: Scene::new(),
            is_pressed: false,
        }
    }
}

impl AnimationTab {
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

    pub fn left_button_pressed(&self, position: Point) {
        //Nothing for now
    }

    pub fn sim_selected(&mut self, result: &MultibodyResult) {
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
                            cuboid.length,
                            cuboid.width,
                            cuboid.height,
                            rotation,
                            position,
                        );
                        self.scene.cuboids.push(cuboid);
                    }
                }
            }
        }
        dbg!(&self.scene.cuboids);
    }
}
