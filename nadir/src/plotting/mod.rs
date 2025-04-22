use figure::Figure;
use iced::{
    Point, Rectangle, Renderer, Size, Theme,
    event::Status,
    mouse::{self, Button, Cursor, ScrollDelta},
    widget::canvas::{Cache, Event, Geometry, Program},
    window::Id,
};
use std::time::Instant;
use theme::{PlotTheme, PlotThemes};
use thiserror::Error;

use crate::window_manager::Message;

mod axes;
mod axis;
pub mod figure;
mod legend;
mod line;
mod note_bar;
mod series;
mod theme;
mod title_bar;

#[derive(Debug, Error)]
pub enum PlotErrors {
    #[error("x data and y data lengths must match")]
    DataSizeMismatch,
}

#[derive(Debug, Clone)]
pub enum PlotMessage {
    // Message::CursorMoved(position) => self.canvas.cursor_moved(point),
    Tick(Instant),
    MouseLeftPressed(Point),
    MouseLeftReleased(Point),
    WheelScrolled(Point, ScrollDelta),
    WindowResized(Size),
}

#[derive(Debug)]
pub struct PlotProgram {
    id: Option<Id>,
    figure: Figure,
    cache: Cache,
    theme: PlotThemes,
}

impl PlotProgram {
    pub fn mouse_left_clicked(&mut self, point: Point) {
        self.figure.mouse_left_clicked(point);
        self.cache.clear();
    }

    pub fn mouse_left_released(&mut self, point: Point) {
        self.figure.mouse_left_released(point);
        self.cache.clear();
    }

    pub fn new() -> Self {
        PlotProgram {
            id: None,
            figure: Figure::new(),
            cache: Cache::new(),
            theme: PlotThemes::Dark,
        }
    }

    pub fn set_window_id(&mut self, id: Id) {
        self.id = Some(id);
    }

    pub fn update(&mut self, message: PlotMessage) {
        match message {
            // Message::CursorMoved(position) => self.canvas.cursor_moved(point),
            PlotMessage::Tick(instant) => {} //self.tick(instant),
            PlotMessage::MouseLeftPressed(point) => self.mouse_left_clicked(point),
            PlotMessage::MouseLeftReleased(point) => self.mouse_left_released(point),
            PlotMessage::WheelScrolled(position, delta) => self.wheel_scrolled(position, delta),
            PlotMessage::WindowResized(size) => self.window_resized(size),
        }
    }

    pub fn window_resized(&mut self, window_size: Size) {
        self.figure.window_resized(window_size);
        self.cache.clear();
    }

    pub fn wheel_scrolled(&mut self, point: Point, delta: ScrollDelta) {
        self.figure.wheel_scrolled(point, delta);
        self.cache.clear();
    }
}

impl Program<Message> for PlotProgram {
    type State = Cache;

    fn draw(
        &self,
        state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: Cursor,
    ) -> Vec<Geometry> {
        let content = self.cache.draw(renderer, bounds.size(), |frame| {
            // background
            frame.fill_rectangle(
                Point::ORIGIN,
                frame.size(),
                self.theme.palette().dark_background,
            );

            // // axes
            // for axes in &self.axes {
            //     frame.with_save(|frame| {
            //         // move to axis location
            //         frame.translate(axes.bounds.position() - Point::ORIGIN);
            //         // scale down to axes size
            //         let x_scale = axes.bounds.width / frame.width();
            //         let y_scale = axes.bounds.height / frame.height();
            //         frame.scale_nonuniform(Vector::new(x_scale, y_scale));
            //         axes.draw(frame, &state.theme);
            //     });
            // }
        });
        vec![content]
    }

    fn update(
        &self,
        _state: &mut Self::State,
        event: Event,
        _bounds: Rectangle,
        cursor: Cursor,
    ) -> (Status, Option<Message>) {
        match event {
            Event::Mouse(event) => match event {
                mouse::Event::WheelScrolled { delta } => {
                    if let Some(point) = cursor.position() {
                        (Status::Captured, Some(Message::WheelScrolled(delta)))
                    } else {
                        (Status::Captured, None)
                    }
                }
                mouse::Event::ButtonPressed(button) => match button {
                    Button::Left => {
                        if let Some(point) = cursor.position() {
                            (
                                Status::Captured,
                                Some(Message::PlotMessage(PlotMessage::MouseLeftPressed(point))),
                            )
                        } else {
                            (Status::Captured, None)
                        }
                    }
                    _ => (Status::Ignored, None),
                },
                mouse::Event::ButtonReleased(button) => match button {
                    Button::Left => {
                        if let Some(point) = cursor.position() {
                            (
                                Status::Captured,
                                Some(Message::PlotMessage(PlotMessage::MouseLeftReleased(point))),
                            )
                        } else {
                            (Status::Captured, None)
                        }
                    }
                    _ => (Status::Captured, None),
                },
                // mouse::Event::CursorMoved { position } => {
                //     (Status::Captured, Some(Message::CursorMoved(position)))
                // }
                _ => (Status::Captured, None),
            },
            _ => (Status::Ignored, None),
        }
    }
}
