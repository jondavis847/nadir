use iced::mouse::ScrollDelta;
use iced::widget::canvas;
use iced::window::icon;

use crate::canvas::PlotCanvas;
use crate::SeriesMap;
use iced::{window, Settings};
use iced::{Element, Fill, Size, Subscription};
use iced::{Point, Task};
use std::time::Instant;

const ICON_BYTES: &[u8] = include_bytes!("../resources/nadir.png");

pub fn main(series: SeriesMap) -> iced::Result {
    // let manifest_dir = std::env::var("CARGO_MANIFEST_DIR").unwrap();
    // let icon_path = PathBuf::from(manifest_dir).join("resources/nasa_aquamarine.png");
    //let icon_path = Path::new("./resources/nasa_aquamarine.png");
    let (icon_rgba, icon_width, icon_height) = {
        // let image = image::open(icon_path)
        //     .expect("Failed to open icon path")
        //     .into_rgba8();
        // let (width, height) = image.dimensions();
        let image = image::load_from_memory(ICON_BYTES)
        .expect("Failed to load icon from memory")
        .into_rgba8();

    let (width, height) = image.dimensions();
    //let icon_rgba = image.into_raw();
        (image.into_raw(), width, height)
    };

    let icon = icon::from_rgba(icon_rgba, icon_width, icon_height).unwrap();
    let settings = Settings {
        id: Some("plot_window".into()),
        antialiasing: true,
        ..Default::default()
    };
    let window_settings = window::Settings {
        size: Size::new(700.0, 350.0),
        icon: Some(icon),
        ..Default::default()
    };

    iced::application("NADIR Plot", PlotApp::update, PlotApp::view)
        .subscription(PlotApp::subscription)
        .centered()
        .settings(settings)
        .window(window_settings)
        .run_with(|| PlotApp::new(series))
}

pub struct PlotApp {
    state: AppState,
    canvas: PlotCanvas,
}

#[derive(Debug, Clone, Copy)]
pub enum Message {
    Tick(Instant),
    WheelScrolled(Point, ScrollDelta),
}

impl PlotApp {
    fn new(series: SeriesMap) -> (Self, Task<Message>) {
        (
            Self {
                state: AppState::default(),
                canvas: PlotCanvas::new(&series),
            },
            window::get_latest().and_then(window::gain_focus),
        )
    }
    fn update(&mut self, message: Message) {
        match message {
            Message::Tick(instant) => {
                self.state.update(instant);
            }
            Message::WheelScrolled(position, delta) => {
                self.canvas.wheel_scrolled(position, delta);
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
