use crate::ui::theme::ButtonStyles;
use crate::Message;
use iced::{
    widget::{button, text, Column},
    Element, Length,
};

use std::collections::HashMap;

#[derive(Debug)]
pub struct PlotSimMenu {
    options: HashMap<String, SimMenuOption>,
}

impl Default for PlotSimMenu {
    fn default() -> Self {
        Self {
            options: HashMap::new(),
        }
    }
}

impl PlotSimMenu {
    pub fn add_option(&mut self, option: SimMenuOption) {
        self.options.insert(option.label.clone(), option);
    }

    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let mut plot_sim_menu = Column::new().width(Length::FillPortion(1));
        for (_, option) in &self.options {
            plot_sim_menu = plot_sim_menu.push(option.content());
        }
        plot_sim_menu.into()
    }

    pub fn sim_selected(&mut self, sim_name: &str) {
        let option = self.options.get_mut(sim_name).unwrap();
        option.is_selected = !option.is_selected;
    }
}

#[derive(Debug, Default)]
pub struct SimMenuOption {
    label: String,
    is_selected: bool,
}

impl SimMenuOption {
    pub fn new(label: String) -> Self {
        Self {
            label: label,
            is_selected: false,
        }
    }
    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let style = match self.is_selected {
            true => ButtonStyles::Selected,
            false => ButtonStyles::Default,
        };

        button(text(self.label.clone()))
            .on_press(Message::SimSelected(self.label.clone()))
            .width(Length::Fill)
            .style(style)
            .into()
    }
}
