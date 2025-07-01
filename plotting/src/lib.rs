use figure::Figure;
use iced::{
    Element, Length, Point, Rectangle, Renderer, Size, Subscription, Task, Theme,
    event::Status,
    keyboard,
    mouse::{self, Button, Cursor, ScrollDelta},
    widget::{
        canvas,
        canvas::{Cache, Event, Geometry, Program},
        column,
    },
    window::{self, Id, icon},
};
use std::{
    sync::{Arc, Mutex},
    time::Instant,
};
use theme::{PlotTheme, PlotThemes};
use thiserror::Error;

pub mod axes;
mod axis;
pub mod datatip;
pub mod figure;
pub mod legend;
pub mod line;
mod note_bar;
pub mod series;
pub mod theme;
mod title_bar;

#[derive(Debug, Error)]
pub enum PlotErrors {
    #[error("axes already at that position")]
    AxesAlreadyInThatPosition,
    #[error("axes index out of bounds")]
    AxesIndexOOB,
    #[error("x data and y data lengths must match")]
    DataSizeMismatch,
    #[error("alpha must be between 0.0 and 1.0")]
    InvalidAlpha,
    #[error("line width must be > 0.0")]
    SmallLineWidth,
}

#[derive(Debug, Clone)]
pub enum Message {
    CursorMoved(Point),
    MouseLeftPressed(Point),
    MouseLeftReleased(Point),
    MouseMiddlePressed(Point),
    MouseMiddleReleased(Point),
    Tick(Instant),
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
    pub fn clear_cache(&mut self) {
        self.cache
            .clear();
    }

    pub fn cursor_moved(&mut self, position: Point) {
        let mut figure = self
            .figure
            .lock()
            .unwrap();
        figure.cursor_moved(position);
        self.cache
            .clear();
    }

    pub fn mouse_left_clicked(&mut self, point: Point) {
        let mut figure = self
            .figure
            .lock()
            .unwrap();
        // determine if single click or double click
        let current_click = Instant::now();
        if current_click
            .duration_since(self.last_click)
            .as_secs_f32()
            < 0.5
        {
            // double click
            figure.mouse_double_clicked(point);
        } else {
            //single click
            figure.mouse_left_clicked(point);
        }
        self.last_click = current_click;
        self.cache
            .clear();
    }

    pub fn mouse_left_released(&mut self, point: Point) {
        let figure = &mut *self
            .figure
            .lock()
            .unwrap();
        figure.mouse_left_released(point);
        self.cache
            .clear();
    }

    pub fn mouse_middle_clicked(&mut self, point: Point) {
        let figure = &mut *self
            .figure
            .lock()
            .unwrap();
        figure.mouse_middle_clicked(point);
        self.cache
            .clear();
    }

    pub fn mouse_middle_released(&mut self, point: Point) {
        let figure = &mut *self
            .figure
            .lock()
            .unwrap();
        figure.mouse_middle_released(point);
        self.cache
            .clear();
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
        let figure = &mut *self
            .figure
            .lock()
            .unwrap();
        figure.set_window_id(id);
    }

    pub fn tick(&mut self, instant: Instant) {
        let mut figure = self
            .figure
            .lock()
            .unwrap();
        let dt = instant
            .duration_since(self.last_tick)
            .as_secs_f32();
        let clear = figure.animation_tick(dt);
        self.last_tick = instant;
        if clear {
            self.cache
                .clear();
        }
    }

    // these are just from canvas events
    // there are additional events handled at the application level
    pub fn process_message(&mut self, message: Message) {
        match message {
            Message::CursorMoved(point) => self.cursor_moved(point),
            Message::MouseLeftPressed(point) => self.mouse_left_clicked(point),
            Message::MouseLeftReleased(point) => self.mouse_left_released(point),
            Message::MouseMiddlePressed(point) => self.mouse_middle_clicked(point),
            Message::MouseMiddleReleased(point) => self.mouse_middle_released(point),
            Message::WheelScrolled(position, delta) => self.wheel_scrolled(position, delta),
            Message::WindowResized(size) => self.window_resized(size),
            Message::Tick(instant) => self.tick(instant),
        }
    }

    pub fn window_resized(&mut self, window_size: Size) {
        let mut figure = self
            .figure
            .lock()
            .unwrap();
        figure.window_resized(window_size);
        self.cache
            .clear();
    }

    pub fn wheel_scrolled(&mut self, point: Point, delta: ScrollDelta) {
        let mut figure = self
            .figure
            .lock()
            .unwrap();
        figure.wheel_scrolled(point, delta);
        self.cache
            .clear();
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
        let content = self
            .cache
            .draw(
                renderer,
                bounds.size(),
                |frame| {
                    let figure = self
                        .figure
                        .lock()
                        .unwrap();
                    figure.draw(frame, &self.theme);
                },
            );
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
                mouse::Event::ButtonPressed(button) => match button {
                    Button::Left => {
                        if let Some(point) = cursor.position() {
                            (
                                Status::Captured,
                                Some(Message::MouseLeftPressed(
                                    point,
                                )),
                            )
                        } else {
                            (Status::Captured, None)
                        }
                    }
                    Button::Middle => {
                        if let Some(point) = cursor.position() {
                            (
                                Status::Captured,
                                Some(Message::MouseMiddlePressed(
                                    point,
                                )),
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
                                Some(Message::MouseLeftReleased(
                                    point,
                                )),
                            )
                        } else {
                            (Status::Captured, None)
                        }
                    }
                    Button::Middle => {
                        if let Some(point) = cursor.position() {
                            (
                                Status::Captured,
                                Some(Message::MouseMiddleReleased(
                                    point,
                                )),
                            )
                        } else {
                            (Status::Captured, None)
                        }
                    }
                    _ => (Status::Captured, None),
                },
                mouse::Event::CursorMoved { position } => (
                    Status::Captured,
                    Some(Message::CursorMoved(position)),
                ),
                mouse::Event::WheelScrolled { delta } => {
                    if let Some(point) = cursor.position() {
                        (
                            Status::Captured,
                            Some(Message::WheelScrolled(
                                point, delta,
                            )),
                        )
                    } else {
                        (Status::Captured, None)
                    }
                }
                _ => (Status::Captured, None),
            },
            _ => (Status::Ignored, None),
        }
    }
}

pub struct QuickPlot {
    program: PlotProgram,
}

impl QuickPlot {
    pub fn new(figure: Figure) -> Self {
        Self { program: PlotProgram::new(Arc::new(Mutex::new(figure))) }
    }

    pub fn plot(figure: Figure) -> iced::Result {
        Self::new(figure).show()
    }

    pub fn show(self) -> iced::Result {
        const ICON_BYTES: &[u8] = include_bytes!("../resources/nadir.png");

        let (icon_rgba, icon_width, icon_height) = {
            let image = image::load_from_memory(ICON_BYTES)
                .expect("Failed to load icon from memory")
                .into_rgba8();

            let (width, height) = image.dimensions();
            (
                image.into_raw(),
                width,
                height,
            )
        };

        let icon = icon::from_rgba(
            icon_rgba,
            icon_width,
            icon_height,
        )
        .unwrap();

        iced::application(
            "nadir",
            Self::update,
            Self::view,
        )
        .subscription(Self::subscription)
        .antialiasing(true)
        .window(window::Settings {
            size: iced::Size::new(720.0, 480.0),
            icon: Some(icon),
            ..Default::default()
        })
        .run_with(move || (self, Task::none()))
    }

    fn subscription(&self) -> Subscription<Message> {
        let iced_events = iced::event::listen_with(|event, _, _id| match event {
            iced::Event::Window(window_event) => match window_event {
                window::Event::Resized(size) => Some(Message::WindowResized(size)),
                _ => None,
            },
            iced::Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => match key {
                _ => None,
            },
            _ => None,
        });

        let animation_tick = window::frames().map(Message::Tick);

        Subscription::batch(vec![
            iced_events,
            animation_tick,
        ])
    }

    fn update(&mut self, message: Message) -> Task<Message> {
        self.program
            .process_message(message);
        Task::none()
    }

    fn view(&self) -> Element<Message> {
        column![
            canvas(&self.program)
                .height(Length::Fill)
                .width(Length::Fill)
        ]
        .into()
    }
}
