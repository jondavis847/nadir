use iced::window::icon;
use plotters::prelude::*;
use plotters_iced::{Chart, ChartBuilder, ChartWidget, DrawingBackend};

use crate::SeriesMap;
use iced::{window, Length, Settings};
use iced::{Element, Fill, Size, Subscription};
use iced::{Point, Task};
use std::time::Instant;

const ICON_BYTES: &[u8] = include_bytes!("../resources/nadir.png");

pub fn main(series: Vec<SeriesMap>) -> iced::Result {
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

    let window_size = Size::new(700.0, 350.0);
    let window_settings = window::Settings {
        size: window_size,
        icon: Some(icon),
        ..Default::default()
    };

    iced::application("NADIR Plot", PlotApp::update, PlotApp::view)
        .subscription(PlotApp::subscription)
        .centered()
        .settings(settings)
        .window(window_settings)
        .run_with(move || PlotApp::new(series, window_size))
}

pub struct PlotApp {
    state: AppState,
    series_map: Vec<SeriesMap>,
}

impl Chart<Message> for PlotApp {
    type State = ();

    fn build_chart<DB: DrawingBackend>(&self, _state: &Self::State, mut builder: ChartBuilder<DB>) {
        let x_range = self.series_map.xmin..self.series_map.xmax;
        let y_range = self.series_map.ymin..self.series_map.ymax;

        let mut chart = builder
            .x_label_area_size(30)
            .y_label_area_size(30)
            .margin(10)
            .build_cartesian_2d(x_range, y_range)
            .expect("Failed to build chart");

        chart
            .configure_mesh()
            .x_labels(5)
            .y_labels(5)
            .draw()
            .expect("Failed to draw mesh");

        for series in &self.series_map.map {
            let data: Vec<(f32, f32)> = series
                .points
                .iter()
                .map(|p| (p.x, p.y))
                .collect();

            chart
                .draw_series(LineSeries::new(data, &RED))
                .expect("Failed to draw series")
                .label(&series.y_name)
                .legend(|(x, y)| PathElement::new(vec![(x, y), (x + 20, y)], &RED));
        }

        chart
            .configure_series_labels()
            .border_style(&BLACK)
            .draw()
            .expect("Failed to draw legend");
    }
}

#[derive(Debug, Clone, Copy)]
pub enum Message {
    Tick(Instant),
    // MouseLeftPressed(Point),
    // MouseLeftReleased(Point),
    // WheelScrolled(Point, ScrollDelta),
    // WindowResized(Size),
}

impl PlotApp {
    fn new(series: Vec<SeriesMap>, window_size: Size) -> (Self, Task<Message>) {
        (
            Self {
                state: AppState::default(),
                series_map: series,
            },
            window::get_latest().and_then(window::gain_focus),
        )
    }
    fn update(&mut self, message: Message) {
        match message {
            Message::Tick(instant) => self.state.update(instant),
        //     Message::MouseLeftPressed(point) => self.canvas.mouse_left_clicked(point),
        //     Message::MouseLeftReleased(point) => self.canvas.mouse_left_released(point),
        //     Message::WheelScrolled(position, delta) => self.canvas.wheel_scrolled(position, delta),
        //     Message::WindowResized(size) => self.canvas.window_resized(size),
        }
    }

    fn view(&self) -> Element<Message> {
        ChartWidget::new(self)
            .width(Length::Fixed(200.0))
            .height(Length::Fixed(200.0))
            .into()
    }

    fn subscription(&self) -> Subscription<Message> {
        // Subscription for frame ticks
        // let frame_ticks =
        //     iced::time::every(std::time::Duration::from_millis(16)).map(|_| Message::Tick);
        let frame_ticks = window::frames().map(Message::Tick);
        // Subscription for window resize events
        // let window_resizes =
        //     window::resize_events().map(|(_id, size)| Message::WindowResized(size));

        // Combine both subscriptions
        Subscription::batch(vec![frame_ticks])//, window_resizes])
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
