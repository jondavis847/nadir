use figure::Figure;
use iced::{
    Point, Rectangle, Renderer, Size, Theme, Vector,
    event::Status,
    mouse::{self, Button, Cursor, ScrollDelta},
    widget::canvas::{Cache, Event, Geometry, Program},
    window::Id,
};
use std::{
    sync::{Arc, Mutex},
    time::Instant,
};
use theme::{PlotTheme, PlotThemes};
use thiserror::Error;

use crate::window_manager::Message;

pub mod axes;
mod axis;
pub mod figure;
pub mod legend;
pub mod line;
mod note_bar;
pub mod series;
mod theme;
mod title_bar;

#[derive(Debug, Error)]
pub enum PlotErrors {
    #[error("axes already at that position")]
    AxesAlreadyInThatPosition,
    #[error("axes index out of bounds")]
    AxesIndexOOB,
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
    pub figure: Arc<Mutex<Figure>>,
    cache: Cache,
    theme: PlotTheme,
    last_tick: Instant,
    last_click: Instant,
}

impl PlotProgram {
    pub fn tick(&mut self, instant: Instant) {
        let mut figure = self.figure.lock().unwrap();
        let dt = instant.duration_since(self.last_tick).as_secs_f32();
        let clear = figure.animation_tick(dt);
        self.last_tick = instant;
        if clear {
            self.cache.clear();
        }
    }
    pub fn clear_cache(&mut self) {
        self.cache.clear();
    }

    pub fn cursor_moved(&mut self, point: Point) {
        let figure = &mut *self.figure.lock().unwrap();
        figure.cursor_moved(point);
        self.cache.clear();
    }

    pub fn mouse_left_clicked(&mut self, point: Point) {
        let figure = &mut *self.figure.lock().unwrap();
        // determine if single click or double click
        let current_click = Instant::now();
        if current_click.duration_since(self.last_click).as_secs_f32() < 0.5 {
            // double click
            figure.mouse_double_clicked(point);
        } else {
            //single click
            figure.mouse_left_clicked(point);
        }
        self.last_click = current_click;
        self.cache.clear();
    }

    pub fn mouse_left_released(&mut self, point: Point) {
        let figure = &mut *self.figure.lock().unwrap();
        figure.mouse_left_released(point);
        self.cache.clear();
    }

    pub fn mouse_middle_clicked(&mut self, point: Point) {
        let figure = &mut *self.figure.lock().unwrap();
        figure.mouse_middle_clicked(point);
        self.cache.clear();
    }

    pub fn mouse_middle_released(&mut self, point: Point) {
        let figure = &mut *self.figure.lock().unwrap();
        figure.mouse_middle_released(point);
        self.cache.clear();
    }

    pub fn new(figure: Arc<Mutex<Figure>>) -> Self {
        PlotProgram {
            id: None,
            figure,
            cache: Cache::new(),
            theme: PlotThemes::Dark.palette(),
            last_tick: Instant::now(),
            last_click: Instant::now(),
        }
    }

    pub fn set_window_id(&mut self, id: Id) {
        self.id = Some(id);
        let figure = &mut *self.figure.lock().unwrap();
        figure.set_window_id(id);
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
        let figure = &mut *self.figure.lock().unwrap();
        figure.window_resized(window_size);
        self.cache.clear();
    }

    pub fn wheel_scrolled(&mut self, point: Point, delta: ScrollDelta) {
        let figure = &mut *self.figure.lock().unwrap();
        figure.wheel_scrolled(point, delta);
        self.cache.clear();
    }
}

impl Program<Message> for PlotProgram {
    type State = ();

    fn draw(
        &self,
        _state: &Self::State,
        renderer: &Renderer,
        _theme: &Theme,
        bounds: Rectangle,
        _cursor: Cursor,
    ) -> Vec<Geometry> {
        let content = self.cache.draw(renderer, bounds.size(), |frame| {
            // // make sure frame starts at origin
            // frame.translate(Point::ORIGIN - frame.center());
            // // make sure the frame is the right size
            // frame.scale_nonuniform(Vector::new(
            //     bounds.width / frame.width(),
            //     bounds.height / frame.height(),
            // ));
            let figure = &*self.figure.lock().unwrap();
            figure.draw(frame, &self.theme);
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
                        (Status::Captured, Some(Message::WheelScrolled(point, delta)))
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
                    Button::Middle => {
                        if let Some(point) = cursor.position() {
                            (Status::Captured, Some(Message::MouseMiddlePressed(point)))
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
                    Button::Middle => {
                        if let Some(point) = cursor.position() {
                            (Status::Captured, Some(Message::MouseMiddleReleased(point)))
                        } else {
                            (Status::Captured, None)
                        }
                    }
                    _ => (Status::Captured, None),
                },
                mouse::Event::CursorMoved { position } => {
                    (Status::Captured, Some(Message::CursorMoved(position)))
                }
                _ => (Status::Captured, None),
            },
            _ => (Status::Ignored, None),
        }
    }
}
