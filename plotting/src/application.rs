use iced::widget::canvas;
use iced::widget::canvas::{
    stroke::{self, Stroke},
    Geometry, Path,
};
use iced::{mouse, Task};
use iced::{window, Settings};
use iced::{Color, Element, Fill, Point, Rectangle, Renderer, Size, Subscription, Theme, Vector};

use std::collections::HashMap;
use std::time::Instant;

use crate::canvas::PlotCanvas;
use crate::{Series, SeriesMap};

pub fn main(series: SeriesMap) -> iced::Result {
    let settings = Settings {
        id: Some("plot_window".into()),
        antialiasing: true,
        ..Default::default()
    };

    iced::application("NADIR Plot", PlotApp::update, PlotApp::view)
        .subscription(PlotApp::subscription)
        .theme(PlotApp::theme)
        .window_size(Size::new(600.0, 300.0))
        .centered()
        .settings(settings)
        .run_with(|| PlotApp::new(series))
}

#[derive(Default)]
pub struct PlotApp {
    state: AppState,
    canvas: PlotCanvas,
}

#[derive(Debug, Clone, Copy)]
pub enum Message {
    Tick(Instant),
}

impl PlotApp {
    fn new(series: SeriesMap) -> (Self, Task<Message>) {
        (
            Self {
                state: AppState::default(),
                canvas: PlotCanvas::new(series),
            },
            window::get_latest().and_then(window::gain_focus),
        )
    }
    fn update(&mut self, message: Message) {
        match message {
            Message::Tick(instant) => {
                self.state.update(instant);
            }
        }
    }

    fn view(&self) -> Element<Message> {
        canvas(&self.canvas).width(Fill).height(Fill).into()
    }

    fn theme(&self) -> Theme {
        Theme::Moonfly
    }

    fn subscription(&self) -> Subscription<Message> {
        window::frames().map(Message::Tick)
    }
}

#[derive(Debug)]
struct AppState {
    now: Instant,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            now: Instant::now(),
        }
    }
}

impl AppState {
    pub fn new() -> AppState {
        let now = Instant::now();
        AppState { now }
    }

    pub fn update(&mut self, now: Instant) {
        self.now = now;
    }
}
