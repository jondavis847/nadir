use std::io::Read;

mod animation;
mod mouse;

use animation::{AnimationGui, AnimationState};
use iced::{
    alignment, keyboard,
    mouse::ScrollDelta,
    widget::{container, text, Column},
    window::{self, icon},
    Application, Command, Element, Length, Point, Settings, Size, Subscription, Theme, Vector,
};

use multibody::result::MultibodyResult;
use std::time::Instant;

// Define the possible user interactions
#[derive(Debug, Clone)]
pub enum Message {
    AnimationTick(Instant),
    CameraRotation(Vector),
    ChannelDataReceived,
    //LeftButtonPressed(Point),
    //LeftButtonReleased(Point),
    Loaded(Result<(), String>),
    MiddleButtonPressed(Point),
    RightButtonPressed(Point),
    RightButtonReleased(Point),
    WheelScrolled(ScrollDelta),
    WindowResized(Size),
}

fn main() -> iced::Result {
    // Read the result data from stdin
    let mut buffer = Vec::new();
    std::io::stdin()
        .read_to_end(&mut buffer)
        .expect("Failed to read from stdin");
    // Deserialize the byte array back into a MultibodyResult struct
    let result: MultibodyResult =
        bincode::deserialize(&buffer).expect("Failed to deserialize data");

    // load the icon
    const ICON: &[u8] = include_bytes!("../resources/icon.png");
    let icon_image = image::load_from_memory(ICON).expect("Failed to load icon");
    let icon_rgba = icon_image.to_rgba8();
    let (icon_width, icon_height) = icon_rgba.dimensions();
    let icon = icon::from_rgba(icon_rgba.into_vec(), icon_width, icon_height).unwrap();

    let mut settings = Settings::with_flags(result);
    settings.antialiasing = true;
    settings.window.size = Size::new(1280.0, 720.0);
    settings.window.icon = Some(icon);

    AnimationGui::run(settings)
}

async fn load() -> Result<(), String> {
    // load textures here
    Ok(())
}

impl Application for AnimationGui {
    type Message = Message;
    type Theme = Theme;
    type Executor = iced::executor::Default;
    type Flags = MultibodyResult;

    fn new(flags: MultibodyResult) -> (Self, Command<Self::Message>) {
        let mut state = AnimationState::default();
        state.initialize(&flags);
        (
            Self {
                state,
                result: Some(flags),
            },
            Command::perform(load(), Message::Loaded),
        )
    }

    fn title(&self) -> String {
        String::from("GADGT")
    }

    fn update(&mut self, message: Message) -> Command<Message> {
        let state = &mut self.state;

        match message {
            Message::AnimationTick(instant) => {
                state.animate(self.result.as_ref().unwrap(), instant);
                Command::none()
            }
            Message::CameraRotation(delta) => state.camera_rotated(delta),
            Message::ChannelDataReceived => Command::none(),
            //Message::LeftButtonPressed(cursor) => state.left_button_pressed(cursor),
            //Message::LeftButtonReleased(cursor) => state.left_button_released(cursor),
            Message::Loaded(_) => {
                state.loaded = true;
                Command::none()
            }
            Message::MiddleButtonPressed(cursor) => state.middle_button_pressed(cursor),
            Message::RightButtonPressed(cursor) => state.right_button_pressed(cursor),
            Message::RightButtonReleased(cursor) => state.right_button_released(cursor),
            Message::WheelScrolled(delta) => state.wheel_scrolled(delta),
            Message::WindowResized(_size) => Command::none(), //state.window_resized(size),
        }
    }

    fn view(&self) -> Element<Message, Theme> {
        let surface = Column::new().width(Length::Fill).height(Length::Fill);

        let surface = match self.state.loaded {
            false => surface.push(loading_view()),
            true => surface.push(loaded_view(&self.state)),
        };

        surface.into()
    }

    fn subscription(&self) -> Subscription<Self::Message> {
        Subscription::batch(vec![
            window::frames().map(Message::AnimationTick),
            iced::event::listen_with(|event, _| match event {
                iced::Event::Window(_, window::Event::Resized { width, height }) => Some(
                    Message::WindowResized(Size::new(width as f32, height as f32)),
                ),
                iced::Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => match key {
                    keyboard::Key::Named(keyboard::key::Named::Enter) => None,
                    keyboard::Key::Named(keyboard::key::Named::Delete) => None,
                    //keyboard::Key::Named(keyboard::key::Named::Tab) => Some(Message::TabPressed),
                    _ => None,
                },
                _ => None,
            }),
        ])
    }

    fn theme(&self) -> Theme {
        Theme::Dark
    }
}
// Helper function to create the loading view
fn loading_view() -> Element<'static, Message, Theme> {
    container(
        text("Loading...")
            .horizontal_alignment(alignment::Horizontal::Center)
            .size(30),
    )
    .width(Length::Fill)
    .height(Length::Fill)
    .center_y()
    .center_x()
    .into()
}

// Helper function to create the main loaded view
fn loaded_view(state: &AnimationState) -> Element<Message, Theme> {
    state.content().into()
}
