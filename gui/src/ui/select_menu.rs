use crate::ui::theme::ButtonStyles;
use crate::Message;
use iced::{
    widget::{button, text, Column},
    Element, Length,
};

use std::collections::HashMap;

#[derive(Debug)]
pub struct SelectMenu {
    options: HashMap<String, SelectMenuOption>,
}

impl Default for SelectMenu {
    fn default() -> Self {
        Self {
            options: HashMap::new(),
        }
    }
}

impl SelectMenu {
    pub fn add_option(&mut self, option_name: String) {
        let option = SelectMenuOption::new(option_name);
        self.options.insert(option.label.clone(), option);
    }

    pub fn content<F>(&self, message: F) -> Element<Message, crate::ui::theme::Theme>
    where
        F: Fn(String) -> Message,
    {
        let mut select_menu = Column::new().width(Length::FillPortion(1));
        for (_, option) in &self.options {
            select_menu = select_menu.push(option.content(&message));
        }
        select_menu.into()
    }

    pub fn get_selected_options(&self) -> Vec<String> {
        self.options
            .iter()
            .filter_map(|(name, option)| {
                if option.is_selected {
                    Some(name.clone())
                } else {
                    None
                }
            })
            .collect()
    }

    pub fn option_selected(&mut self, option_name: &str) {
        let option = self.options.get_mut(option_name).unwrap();
        option.is_selected = !option.is_selected;
    }

    pub fn update_options(&mut self, options: Vec<String>) {
        self.options.clear();
        for option_string in options {
            let option = SelectMenuOption::new(option_string.clone());
            self.options.insert(option_string.clone(), option);
        }
    }
}

#[derive(Debug, Default)]
pub struct SelectMenuOption {
    label: String,
    is_selected: bool,
}

impl SelectMenuOption {
    pub fn new(label: String) -> Self {
        Self {
            label: label,
            is_selected: false,
        }
    }
    pub fn content<F>(&self, message: F) -> Element<Message, crate::ui::theme::Theme>
    where
        F: Fn(String) -> Message,
    {
        let style = match self.is_selected {
            true => ButtonStyles::Selected,
            false => ButtonStyles::Default,
        };

        button(text(self.label.clone()))
            .on_press(message(self.label.clone()))
            .width(Length::Fill)
            .style(style)
            .into()
    }
}
