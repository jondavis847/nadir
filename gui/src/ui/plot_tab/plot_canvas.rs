use iced::{
    mouse::{self, Cursor},
    widget::{
        canvas::{
            self,
            event::{Event, Status},
            Cache, Canvas, Geometry,
        },
        container,
    },
    Element, Length, Point, Rectangle, Renderer,
};

use crate::ui::theme::Theme;
use crate::Message;

pub mod axes;
pub mod line;

use axes::Axes;
use line::Line;

#[derive(Debug, Default)]
pub struct PlotState {
    cache: Cache,
    axes: Axes,
}

#[derive(Debug, Default)]
pub struct PlotCanvas {
    state: PlotState,
}

impl PlotCanvas {
    pub fn content(&self) -> Element<Message, Theme> {
        container(Canvas::new(self).width(Length::Fill).height(Length::Fill))
            .width(Length::FillPortion(8))
            .height(Length::Fill)
            .into()
    }

    #[inline]
    pub fn plot(&mut self, line_label: String, points: Vec<Point>) {        
        self.state.axes.plot(line_label, points);
        self.state.cache.clear();
    }
}

impl canvas::Program<Message, Theme> for PlotCanvas {
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
        renderer: &Renderer,
        theme: &Theme,
        bounds: Rectangle,
        _cursor: mouse::Cursor,
    ) -> Vec<Geometry> {
        let all_content = self.state.cache.draw(renderer, bounds.size(), |frame| {
            self.state.axes.draw(frame, theme);
        });
        vec![all_content]
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
