use crate::Message;
use iced::{
    widget::{button, text, text::Text, text_input, Column, Row},
    Command, Element, Length,
};

#[derive(Debug, Clone)]
pub struct SimDiv {
    pub name: String,
    pub start_time: String,
    pub stop_time: String,
    pub dt: String,
}

impl Default for SimDiv {
    fn default() -> Self {
        Self {
            name: String::new(),
            start_time: String::new(),
            stop_time: String::new(),
            dt: String::new(),
        }
    }
}

impl SimDiv {
    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let create_text_input =
            |label: &str, value: &str, placeholder: &str, on_input: fn(String) -> Message| {
                Row::new()
                    .spacing(10)
                    .push(text(label).width(Length::FillPortion(3)))
                    .push(
                        text_input(placeholder, value)
                            .on_input(on_input)
                            .width(Length::FillPortion(2)),
                    )
                    .width(Length::Fill)
                    .padding(5.0)
            };

        Column::new()
            .push(
                Column::new()
                    .push(create_text_input(
                        "Sim Name",
                        &self.name,
                        "enter name",
                        |string| Message::SimNameChanged(string),
                    ))
                    .push(create_text_input(
                        "Start Time",
                        &self.start_time.to_string(),
                        "0.0",
                        |string| Message::SimStartTimeChanged(string),
                    ))
                    .push(create_text_input(
                        "Stop Time",
                        &self.stop_time.to_string(),
                        "10.0",
                        |string| Message::SimStopTimeChanged(string),
                    ))
                    .push(create_text_input(
                        "Step Size",
                        &self.dt.to_string(),
                        "1.0",
                        |string| Message::SimDtChanged(string),
                    ))
                    .height(Length::FillPortion(19)),
            )
            .push(
                button(
                    Text::new("Simulate")
                        .height(Length::Fill)
                        .width(Length::Fill)
                        .horizontal_alignment(iced::alignment::Horizontal::Center)
                        .vertical_alignment(iced::alignment::Vertical::Center)
                        .style(crate::ui::theme::TextStyles::Primary),
                )
                .on_press(Message::Simulate)
                .height(Length::FillPortion(1))
                .width(Length::Fill),
            )
            .width(Length::FillPortion(1))
            .height(Length::Fill)
            .into()
    }

    pub fn dt_changed(&mut self, value: String) -> Command<Message> {
        self.dt = value;
        Command::none()
    }
    pub fn start_time_changed(&mut self, value: String) -> Command<Message> {
        self.start_time = value;
        Command::none()
    }
    pub fn stop_time_changed(&mut self, value: String) -> Command<Message> {
        self.stop_time = value;
        Command::none()
    }

    pub fn name_changed(&mut self, value: String) -> Command<Message> {
        self.name = value;
        Command::none()
    }
}
