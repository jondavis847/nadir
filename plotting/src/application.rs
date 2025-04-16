use std::{
    io::{self, BufReader, Read},
    sync::{
        Arc, Mutex,
        mpsc::{Receiver, channel},
    },
    thread,
    time::Instant,
};

use iced::{
    Element,
    Length::Fill,
    Point, Settings, Size, Subscription, Task,
    mouse::ScrollDelta,
    window::{self, icon},
};

use crate::{canvas::PlotCanvas, figure::Figure};

impl PlotApp {
    fn new(figure: Figure) -> (Self, Task<Message>) {
        // Spawn a thread to read from stdin
        thread::spawn(move || {
            let stdin = io::stdin();
            let mut reader = BufReader::new(stdin);

            loop {
                // Read message length (4 bytes for u32)
                let mut len_bytes = [0u8; 4];
                if reader.read_exact(&mut len_bytes).is_err() {
                    // Send None to indicate disconnection
                    tx.send(None).unwrap_or(());
                    break;
                }

                // Convert bytes to length
                let msg_len = u32::from_le_bytes(len_bytes) as usize;

                // Read the message data
                let mut msg_data = vec![0u8; msg_len];
                if reader.read_exact(&mut msg_data).is_err() {
                    // Send None to indicate disconnection
                    tx.send(None).unwrap_or(());
                    break;
                }

                // Deserialize the command
                match deserialize(&msg_data) {
                    Ok(cmd) => {
                        if tx.send(Some(cmd)).is_err() {
                            // Channel closed, main app has terminated
                            break;
                        }
                    }
                    Err(_) => {
                        // Just skip malformed commands
                        continue;
                    }
                }
            }
        });
        (
            Self {
                canvas: PlotCanvas::new(figure),
            },
            window::get_latest().and_then(window::gain_focus),
        )
    }
    fn update(&mut self, message: Message) {
        // match message {
        //     // Message::CursorMoved(position) => self.canvas.cursor_moved(point),
        //     Message::Tick(instant) => self.state.update(instant),
        //     Message::MouseLeftPressed(point) => self.canvas.mouse_left_clicked(point),
        //     Message::MouseLeftReleased(point) => self.canvas.mouse_left_released(point),
        //     Message::WheelScrolled(position, delta) => self.canvas.wheel_scrolled(position, delta),
        //     Message::WindowResized(size) => self.canvas.window_resized(size),
        // }
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
        // let window_resizes =
        //     window::resize_events().map(|(_id, size)| Message::WindowResized(size));

        // Combine both subscriptions
        Subscription::batch(vec![frame_ticks]) //, window_resizes])
    }
}

const ICON_BYTES: &[u8] = include_bytes!("../resources/nadir.png");

fn main() -> iced::Result {
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

    iced::application("NADIR Plot", PlotApp::update, PlotApp::view)
        .subscription(PlotApp::subscription)
        .centered()
        .settings(settings)
        .window(window_settings)
        .run_with(move || PlotApp::new(series, window_size))
}
