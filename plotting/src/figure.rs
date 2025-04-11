use iced::mouse::ScrollDelta;
use iced::widget::canvas;
use iced::window::icon;

use crate::axes::Axes;
use crate::canvas::PlotCanvas;
use iced::{Element, Fill, Size, Subscription};
use iced::{Point, Task};
use iced::{Settings, window};
use std::collections::HashMap;
use std::time::Instant;

const ICON_BYTES: &[u8] = include_bytes!("../resources/nadir.png");

pub fn main(id: usize) -> iced::Result {
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

    let window_size = Size::new(800.0, 400.0);
    let window_settings = window::Settings {
        size: window_size,
        icon: Some(icon),
        ..Default::default()
    };

    iced::application("NADIR Plot", Figure::update, Figure::view)
        .subscription(Figure::subscription)
        .centered()
        .settings(settings)
        .window(window_settings)
        .run_with(move || Figure::new(id, window_size))
}

pub struct Figure {
    id: usize,
    state: AppState,
    canvas: PlotCanvas,
    axes: HashMap<u32, Axes>,
}

#[derive(Debug, Clone, Copy)]
pub enum Message {
    // CursorMoved(Point),
    Tick(Instant),
    MouseLeftPressed(Point),
    MouseLeftReleased(Point),
    WheelScrolled(Point, ScrollDelta),
    WindowResized(Size),
}

impl Figure {
    fn new(id: usize, window_size: Size) -> (Self, Task<Message>) {
        (
            Self {
                id,
                axes: HashMap::new(),
                state: AppState::default(),
                canvas: PlotCanvas::new(window_size),
            },
            window::get_latest().and_then(window::gain_focus),
        )
    }
    fn update(&mut self, message: Message) {
        match message {
            // Message::CursorMoved(position) => self.canvas.cursor_moved(point),
            Message::Tick(instant) => self.state.update(instant),
            Message::MouseLeftPressed(point) => self.canvas.mouse_left_clicked(point),
            Message::MouseLeftReleased(point) => self.canvas.mouse_left_released(point),
            Message::WheelScrolled(position, delta) => self.canvas.wheel_scrolled(position, delta),
            Message::WindowResized(size) => self.canvas.window_resized(size),
        }
    }

    fn view(&self) -> Element<Message> {
        canvas(&self.canvas).width(Fill).height(Fill).into()
    }

    fn subscription(&self) -> Subscription<Message> {
        // Subscription for frame ticks
        // let frame_ticks =
        //     iced::time::every(std::time::Duration::from_millis(16)).map(|_| Message::Tick);
        let frame_ticks = window::frames().map(Message::Tick);
        // Subscription for window resize events
        let window_resizes =
            window::resize_events().map(|(_id, size)| Message::WindowResized(size));

        // Combine both subscriptions
        Subscription::batch(vec![frame_ticks, window_resizes])
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
