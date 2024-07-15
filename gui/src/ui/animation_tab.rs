use crate::{ui::select_menu::SelectMenu, Message};
use iced::{widget::Row, Element, Length, Point};

//pub mod plot_canvas;
//use plot_canvas::PlotCanvas;

#[derive(Debug)]
pub struct AnimationTab {
    pub sim_menu: SelectMenu,
    pub selected_sims: Vec<String>,
}

impl Default for AnimationTab {
    fn default() -> Self {
        Self {
            sim_menu: SelectMenu::new(Length::FillPortion(1), false),
            selected_sims: Vec::new(),
        }
    }
}

impl AnimationTab {
    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let sim_menu = self
            .sim_menu
            .content(|string| Message::PlotSimSelected(string));

        Row::new()
            .push(sim_menu)
            .height(Length::FillPortion(15))
            .width(Length::Fill)
            .into()
    }
}
