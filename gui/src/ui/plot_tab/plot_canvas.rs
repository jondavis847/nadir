use iced::{
    mouse::{self, Cursor},
    widget::canvas::{
        self,
        event::{Event, Status},
        Geometry,
    },
    Rectangle, Renderer,
};

use crate::ui::theme::Theme;
use crate::Message;

#[derive(Debug, Default)]
pub struct PlotState {}

#[derive(Debug)]
pub struct PlotCanvas<'a> {
    state: PlotState,
    app_state: &'a crate::AppState,
}

impl<'a> PlotCanvas<'a> {
    pub fn new(app_state: &'a crate::AppState) -> Self {
        Self {
            state: PlotState {},
            app_state: app_state,
        }
    }
}

impl<'a> canvas::Program<Message, Theme> for PlotCanvas<'a> {
    type State = PlotState;

    fn update(
        &self,
        _state: &mut Self::State,
        event: Event,
        bounds: Rectangle,
        cursor: Cursor,
    ) -> (Status, Option<Message>) {
        if !cursor.is_over(bounds) {
            return (Status::Ignored, None);
        };

        let canvas_cursor_position = cursor.position_in(bounds).unwrap();

        match event {
            Event::Mouse(mouse_event) => match mouse_event {
                mouse::Event::ButtonPressed(mouse::Button::Left) => (
                    Status::Captured,
                    Some(Message::LeftButtonPressed(canvas_cursor_position)),
                ),
                mouse::Event::ButtonReleased(mouse::Button::Left) => (
                    Status::Captured,
                    Some(Message::LeftButtonReleased(canvas_cursor_position)),
                ),
                mouse::Event::ButtonPressed(mouse::Button::Middle) => (
                    Status::Captured,
                    Some(Message::MiddleButtonPressed(canvas_cursor_position)),
                ),
                mouse::Event::ButtonPressed(mouse::Button::Right) => (
                    Status::Captured,
                    Some(Message::RightButtonPressed(canvas_cursor_position)),
                ),
                mouse::Event::ButtonReleased(mouse::Button::Right) => (
                    Status::Captured,
                    Some(Message::RightButtonReleased(canvas_cursor_position)),
                ),
                mouse::Event::CursorMoved { position: _ } => (
                    Status::Captured,
                    Some(Message::CursorMoved(canvas_cursor_position)),
                ),
                mouse::Event::WheelScrolled { delta } => {
                    (Status::Captured, Some(Message::WheelScrolled(delta)))
                }
                _ => (Status::Captured, None),
            },
            _ => (Status::Ignored, None),
        }
    }

    fn draw(
        &self,
        _state: &Self::State,
        _renderer: &Renderer,
        _theme: &Theme,
        _bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Geometry> {
        Vec::new()
    }

    fn mouse_interaction(
        &self,
        _state: &Self::State,
        _bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> mouse::Interaction {
        mouse::Interaction::default()
    }
}
