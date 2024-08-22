//#![windows_subsystem = "windows"]
//#![warn(missing_docs)]

mod scene;

use iced::{
    advanced::graphics::core::window::icon,
    alignment, font, keyboard,
    mouse::ScrollDelta,
    time::Instant,
    widget::{canvas::Canvas, container, text, Column, Row},
    window, Application, Command, Element, Length, Point, Settings, Size, Subscription, Vector,
};
use iced_aw::modal;
use rotations::{axes::Axis, euler_angles::EulerSequence};
use std::{env, path::Path};

#[derive(Debug)]
struct AnimationGui {
    result: MultibodyResult
}

#[derive(Debug, Clone)]
pub struct Animator {
    loaded: bool,
    pub current_time: f32,
    start_time: f32,
    end_time: f32,
    speed: f32,
    instant: iced::time::Instant,
}

impl Animator {
    pub fn new(start_time: f32, end_time: f32) -> Self {
        Self {
            loaded: false,
            start_time,
            end_time,
            current_time: start_time,
            speed: 1.0,
            instant: iced::time::Instant::now(),
        }
    }

    pub fn start(&mut self) {
        self.instant = iced::time::Instant::now();
    }

    pub fn update(&mut self, instant: iced::time::Instant) {
        let dt = instant.duration_since(self.instant).as_secs_f32();
        self.current_time += self.speed * dt;
        //rollover by default for now;
        if self.current_time > self.end_time {
            self.current_time = self.start_time;
        }
        self.instant = instant;
    }
}

pub fn animate(result: &MultibodyResult) -> iced::Result {

    let icon_path = Path::new("./resources/icon.png");
    let (icon_rgba, icon_width, icon_height) = {
        let image = image::open(icon_path)
            .expect("Failed to open icon path")
            .into_rgba8();
        let (width, height) = image.dimensions();
        (image.into_raw(), width, height)
    };

    let icon = icon::from_rgba(icon_rgba, icon_width, icon_height).unwrap();

    let mut settings = Settings::default();
    settings.antialiasing = true;
    settings.window.size = Size::new(1280.0, 720.0);
    settings.window.icon = Some(icon);
    AnimationGui::run(settings)
}

// Define the possible user interactions
#[derive(Debug, Clone)]
enum Message {    
    AnimationTick(Instant),
    LeftButtonPressed(Point),
    LeftButtonReleased(Point),
    Loaded(Result<(), String>),
    MiddleButtonPressed(Point),    
    RightButtonPressed(Point),
    RightButtonReleased(Point),    
    WheelScrolled(ScrollDelta),
    WindowResized(Size),
}

#[derive(Debug)]
enum GadgtGui {
    Loading,
    Loaded(AppState),
}

async fn load() -> Result<(), String> {
    // load textures here
    Ok(())
}

impl Application for AnimationGui {
    type Message = Message;
    type Theme = Theme;
    type Executor = iced::executor::Default;
    type Flags = ();

    fn new(_flags: ()) -> (Self, Command<Self::Message>) {
        (
            Self::Loading,
            Command::<Message>::batch(vec![                ,
                Command::perform(load(), Message::Loaded),
            ]),
        )
    }

    fn title(&self) -> String {
        String::from("GADGT")
    }

    fn update(&mut self, message: Message) -> Command<Message> {
        match self {
            GadgtGui::Loading => {
                if let Message::Loaded(_) = message {
                    *self = GadgtGui::Loaded(Animator::default());
                }
                Command::none()
            }
            GadgtGui::Loaded(state) => match message {                
                Message::AnimationTick(instant) => state.animation(instant),
                Message::LeftButtonPressed(cursor) => state.left_button_pressed(cursor),
                Message::LeftButtonReleased(cursor) => state.left_button_released(cursor),
                Message::Loaded(_) => Command::none(),
                Message::MiddleButtonPressed(cursor) => state.middle_button_pressed(cursor),
                Message::RightButtonPressed(cursor) => state.right_button_pressed(cursor),
                Message::RightButtonReleased(cursor) => state.right_button_released(cursor),                
                Message::WheelScrolled(delta) => state.wheel_scrolled(delta),
                Message::WindowResized(size) => state.window_resized(size),
            },
        }
    }

    fn view(&self) -> Element<Message, Theme> {
        match self {
            GadgtGui::Loading => loading_view(),
            GadgtGui::Loaded(state) => loaded_view(state),
        }
    }

    fn subscription(&self) -> Subscription<Self::Message> {
        Subscription::batch(vec![
            window::frames().map(Message::AnimationTick),
            iced::event::listen_with(|event, _| match event {
                iced::Event::Window(_, window::Event::Resized { width, height }) => Some(
                    Message::WindowResized(Size::new(width as f32, height as f32)),
                ),
                iced::Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => match key {
                    keyboard::Key::Named(keyboard::key::Named::Enter) => {
                        Some(Message::EnterPressed)
                    }
                    keyboard::Key::Named(keyboard::key::Named::Delete) => {
                        Some(Message::DeletePressed)
                    }
                    keyboard::Key::Named(keyboard::key::Named::Tab) => Some(Message::TabPressed),
                    _ => None,
                },
                _ => None,
            }),
        ])
    }
}
// Helper function to create the loading view
fn loading_view() -> Element<'static, Message, Theme> {
    container(
        text("Loading...")
            .horizontal_alignment(alignment::Horizontal::Center)
            .size(50),
    )
    .width(Length::Fill)
    .height(Length::Fill)
    .center_y()
    .center_x()
    .into()
}

// Helper function to create the main loaded view
fn loaded_view(state: &AppState) -> Element<Message, Theme> {
    
    
}
