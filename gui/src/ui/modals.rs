use super::dummies::{DummyBase, DummyComponent};
use crate::{ui::theme::Theme, Message};
use iced::{
    widget::{button, Column, Row},
    Element, Length,
};
use iced_aw::widgets::card;
use uuid::Uuid;
#[derive(Debug, Clone, Copy)]
pub struct ActiveModal {
    pub dummy_type: DummyComponent,
    pub component_id: Option<Uuid>, // None if new component, id if editing component
}

impl ActiveModal {
    pub fn new(dummy_type: DummyComponent, component_id: Option<Uuid>) -> Self {
        Self {
            dummy_type,
            component_id,
        }
    }
}

pub fn create_base_modal(_base: &DummyBase) -> Element<Message, Theme> {
    let content = Column::new();
    let footer = Row::new()
        .spacing(10)
        .padding(5)
        .width(Length::Fill)
        .push(
            button("Cancel")
                .width(Length::Fill)
                .on_press(Message::CloseModal),
        )
        .push(
            button("Ok")
                .width(Length::Fill)
                .on_press(Message::SaveComponent),
        );

    //title doesnt work yet
    card("Base Information", content)
        .foot(footer)
        .max_width(500.0)
        .into()
}
