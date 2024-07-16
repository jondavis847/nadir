use crate::{ui::select_menu::SelectMenu, Message};
use iced::{
    widget::{shader, Row},
    Element, Length, Point,
};

pub mod scene;
use scene::Scene;
//use plot_canvas::PlotCanvas;

#[derive(Debug)]
pub struct AnimationTab {
    pub sim_menu: SelectMenu,
    pub selected_sims: Vec<String>,
    pub scene: Scene,
}

impl Default for AnimationTab {
    fn default() -> Self {
        Self {
            sim_menu: SelectMenu::new(Length::FillPortion(1), false),
            selected_sims: Vec::new(),
            scene: Scene::new(),
        }
    }
}

impl AnimationTab {
    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let sim_menu = self
            .sim_menu
            .content(|string| Message::PlotSimSelected(string));

        let shader_program = shader(&self.scene)
            .width(Length::FillPortion(15))
            .height(Length::Fill);

        Row::new()
            .push(sim_menu)
            .push(shader_program)
            .height(Length::FillPortion(15))
            .width(Length::Fill)
            .into()
    }
}
