use crate::Message;
use iced::{
    widget::{button, text, text::Text, text_input, Column, Row},
    Command, Element, Length, Point, Rectangle, Size,
};

#[derive(Debug, Clone)]
pub struct SimDivState {
    pub start_time: f64,
    pub stop_time: f64,
    pub dt: f64,
}

impl Default for SimDivState {
    fn default() -> Self {
        Self {
            start_time: 0.0,
            stop_time: 10.0,
            dt: 1.0,
        }
    }
}

#[derive(Debug, Clone)]
pub struct SimDiv {
    pub state: SimDivState,
}

impl Default for SimDiv {
    fn default() -> Self {
        let state = SimDivState::default();
        Self { state }
    }
}

impl SimDiv {
    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let create_text_input = |label: &str, value: &str, on_input: fn(String) -> Message| {
            Row::new()
                .spacing(10)
                .push(text(label).width(Length::FillPortion(3)))
                .push(
                    text_input(label, value)
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
                        "start time",
                        &self.state.start_time.to_string(),
                        Message::SimStartTimeChanged,
                    ))
                    .push(create_text_input(
                        "stop time",
                        &self.state.stop_time.to_string(),
                        Message::SimStopTimeChanged,
                    ))
                    .push(create_text_input(
                        "dt",
                        &self.state.dt.to_string(),
                        Message::SimDtChanged,
                    ))
                    .height(Length::FillPortion(19)),
            )
            .push(
                button(
                    Text::new("Simulate")
                        .height(Length::Fill)
                        .width(Length::Fill)
                        .horizontal_alignment(iced::alignment::Horizontal::Center)
                        .vertical_alignment(iced::alignment::Vertical::Center),
                )
                .on_press(Message::Simulate)
                .height(Length::FillPortion(1))
                .width(Length::Fill),
            )
            .into()
    }

    pub fn dt_changed(&mut self, value: String) -> Command<Message> {
        self.state.dt = value.parse().unwrap_or(0.0);
        Command::none()
    }
    pub fn start_time_changed(&mut self, value: String) -> Command<Message> {
        self.state.start_time = value.parse().unwrap_or(0.0);
        Command::none()
    }
    pub fn stop_time_changed(&mut self, value: String) -> Command<Message> {
        self.state.stop_time = value.parse().unwrap_or(0.0);
        Command::none()
    }
}
