mod mouse;
use iced::{
    alignment, keyboard,
    mouse::ScrollDelta,
    widget::{container, text, Column},
    window::{self, icon},
    Application, Command, Element, Length, Point, Settings, Size, Subscription, Theme,
};
use mouse::MouseProcessor;
use multibody::result::MultibodyResult;
use std::{path::Path, time::Instant};

#[derive(Debug, Clone)]
struct AnimationGui {
    state: Option<AnimationState>,
}

#[derive(Debug, Clone)]
struct AnimationState {
    loaded: bool,
    current_time: f32,
    start_time: f32,
    end_time: f32,
    speed: f32,
    instant: iced::time::Instant,
    mouse: MouseProcessor,
}

impl AnimationState {
    pub fn new(result: &MultibodyResult) -> Self {
        Self {
            loaded: false,
            current_time: result.sim_time[0] as f32,
            start_time: result.sim_time[0] as f32,
            end_time: result.sim_time[result.sim_time.len() - 1] as f32,
            speed: 1.0,
            instant: Instant::now(),
            mouse: MouseProcessor::default(),
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

// Define the possible user interactions
#[derive(Debug, Clone)]
enum Message {
    AnimationTick(Instant),
    ChannelDataReceived,
    //LeftButtonPressed(Point),
    //LeftButtonReleased(Point),
    Loaded(Result<(), String>),    
    //MiddleButtonPressed(Point),
    //RightButtonPressed(Point),
    //RightButtonReleased(Point),
    //WheelScrolled(ScrollDelta),
    WindowResized(Size),
}

pub fn main() -> iced::Result {
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
            Self { state: None },
            Command::perform(load(), Message::Loaded),
        )
    }

    fn title(&self) -> String {
        String::from("GADGT")
    }

    fn update(&mut self, message: Message) -> Command<Message> {
        if let Some(state) = &mut self.state {
            match state.loaded {
                false => {
                    // TODO: load required textures for animation
                    Command::none()
                }
                true => match message {
                    Message::AnimationTick(instant) => Command::none(), //state.animate(instant),
                    Message::ChannelDataReceived => Command::none(),
                    //Message::LeftButtonPressed(cursor) => state.left_button_pressed(cursor),
                    //Message::LeftButtonReleased(cursor) => state.left_button_released(cursor),
                    Message::Loaded(_) => Command::none(),
                    
                    //Message::MiddleButtonPressed(cursor) => state.middle_button_pressed(cursor),
                    //Message::RightButtonPressed(cursor) => state.right_button_pressed(cursor),
                    //Message::RightButtonReleased(cursor) => state.right_button_released(cursor),
                    //Message::WheelScrolled(delta) => state.wheel_scrolled(delta),
                    Message::WindowResized(size) => Command::none(), //state.window_resized(size),
                },
            }
        } else {
            Command::none()
        }
    }

    fn view(&self) -> Element<Message, Theme> {
        let surface = Column::new().width(Length::Fill).height(Length::Fill);

        let surface = if let Some(state) = &self.state {
            match state.loaded {
                false => surface.push(loading_view()),
                true => surface.push(loaded_view(state)),
            }
        } else {
            surface
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
fn loaded_view(state: &AnimationState) -> Element<Message, Theme> {
    Column::new().into()
}
