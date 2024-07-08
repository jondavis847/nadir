use iced::{
    widget::{button::Button, text::Text, Column},
    Element,
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
    pub fn content(&self, sims: Vec<String>) -> Element<Message, crate::ui::theme::Theme> {
        let mut content = Column::new();
        for sim in sims {
            let label = Text::new(sim.clone());
            content = content.push(Button::new(label).on_press(Message::SimSelected(sim.clone())));
        }
        content.into()
    }
}

#[derive(Debug, Default)]
pub struct SimMenuButton {
    label: String,
    is_selected: bool,
}
