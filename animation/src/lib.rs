use std::path::PathBuf;

pub mod animation;
mod celestial_animation;
//mod mouse;

use animation::AnimationGui;
use iced::{
    Point, Size, Vector,
    mouse::ScrollDelta,
    window::{self, icon},
};
use std::time::Instant;

#[derive(Debug)]
pub enum AnimationErrors {}

// Define the possible user interactions
#[derive(Debug, Clone)]
pub enum Message {
    AnimationTick(Instant),
    CameraRotation(Vector),
    CameraFovChanged(f32),
    ChannelDataReceived,
    EscapePressed,
    //LeftButtonPressed(Point),
    //LeftButtonReleased(Point),
    Loaded,
    PlaybackSpeedChanged(f64),
    MiddleButtonPressed(Point),
    RightButtonPressed(Point),
    RightButtonReleased(Point),
    WheelScrolled(ScrollDelta),
    WindowResized(Size),
}

pub fn main(result_path: Option<PathBuf>) -> iced::Result {
    let pwd = match result_path {
        Some(path) => path,
        None => std::env::current_dir().unwrap(),
    };

    // load the icon
    const ICON: &[u8] = include_bytes!("../resources/icon.png");
    let icon_image = image::load_from_memory(ICON).expect("Failed to load icon");
    let icon_rgba = icon_image.to_rgba8();
    let (icon_width, icon_height) = icon_rgba.dimensions();
    let icon = icon::from_rgba(icon_rgba.into_vec(), icon_width, icon_height).unwrap();

    let mut window_settings = window::Settings::default();
    window_settings.size = Size::new(1280.0, 720.0);
    window_settings.icon = Some(icon);

    iced::application("NADIR", AnimationGui::update, AnimationGui::view)
        .antialiasing(false)
        .window(window_settings)
        .subscription(AnimationGui::subscription)
        .theme(AnimationGui::theme)
        .run_with(move || AnimationGui::new(pwd))
}
