use iced::{
    widget::{button, text, Column},
    Element, Length,
};

use crate::Message;

#[derive(Debug)]
pub struct PlotSimMenu {}

impl Default for PlotSimMenu {
    fn default() -> Self {
        Self {}
    }
}

impl PlotSimMenu {
    pub fn content(&self, sim_names: Vec<String>) -> Element<Message, crate::ui::theme::Theme> {
        //make the loaded sims menu
        let mut loaded_sims_menu = Column::new().width(Length::FillPortion(1));
        for name in sim_names {
            let label = text(name.clone());
            loaded_sims_menu = loaded_sims_menu.push(
                button(label)
                    .on_press(Message::SimSelected(name.clone()))
                    .width(Length::Fill),
            );
        }
        loaded_sims_menu.into()
    }
}

#[derive(Debug, Default)]
pub struct SimMenuButton {
    label: String,
    is_selected: bool,
}
