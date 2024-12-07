use iced::mouse::ScrollDelta;
use iced::widget::canvas;

use crate::canvas::PlotCanvas;
use crate::SeriesMap;
use iced::{Point, Task};
use iced::{window, Settings};
use iced::{Element, Fill, Size, Subscription};
use std::time::Instant;

pub fn main(series: SeriesMap) -> iced::Result {
    let settings = Settings {
        id: Some("plot_window".into()),
        antialiasing: true,
        ..Default::default()
    };

    iced::application("NADIR Plot", PlotApp::update, PlotApp::view)
        .subscription(PlotApp::subscription)
        .window_size(Size::new(600.0, 300.0))
        .centered()
        .settings(settings)
        .run_with(|| PlotApp::new(series))
}

pub struct PlotApp {
    state: AppState,
    canvas: PlotCanvas,
    series: SeriesMap,
}

#[derive(Debug, Clone, Copy)]
pub enum Message {
    Tick(Instant),
    WheelScrolled(Point,ScrollDelta)
}

impl PlotApp {
    fn new(series: SeriesMap) -> (Self, Task<Message>) {
        (
            Self {
                state: AppState::default(),
                canvas: PlotCanvas::new(&series),
                series,
            },
            window::get_latest().and_then(window::gain_focus),
        )
    }
    fn update(&mut self, message: Message) {
        match message {
            Message::Tick(instant) => {
                self.state.update(instant);
            }
            Message::WheelScrolled(position,delta) => {
                self.canvas.wheel_scrolled(position,delta);
            }
        }
    }

    fn view(&self) -> Element<Message> {
        canvas(&self.canvas).width(Fill).height(Fill).into()
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
    pub fn update(&mut self, now: Instant) {
        self.now = now;
    }
}
