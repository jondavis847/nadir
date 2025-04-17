use iced::advanced::subscription;
use iced::futures::channel::mpsc::UnboundedReceiver;
use iced::widget::{button, center, column, horizontal_space, text_input};
use iced::window::{self, Event, Id};
use iced::{Element, Size, Subscription, Task, Vector};

use std::collections::HashMap;

// Define the plot manager commands
#[derive(Debug, Clone)]
pub enum PlotCommand {
    NewFigure,
}

pub struct PlotManager {
    windows: HashMap<window::Id, PlotWindow>,
    command_rx: UnboundedReceiver<PlotCommand>,
}

#[derive(Debug, Clone)]
pub enum Message {
    None,
    ExternalCommand(PlotCommand),
    OpenWindow,
    WindowOpened(window::Id),
    WindowClosed(window::Id),
}

impl PlotManager {
    pub fn new(command_rx: UnboundedReceiver<PlotCommand>) -> Self {
        Self {
            windows: HashMap::new(),
            command_rx,
        }
    }

    pub fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::None => return Task::none(),
            Message::ExternalCommand(command) => {
                match command {
                    PlotCommand::NewFigure => {
                        // Return a task that will emit the OpenWindow message
                        Task::perform(async {}, |_| Message::OpenWindow)
                    }
                }
            }
            Message::OpenWindow => {
                let Some(last_window) = self.windows.keys().last() else {
                    return Task::none();
                };

                window::get_position(*last_window)
                    .then(|last_position| {
                        let position =
                            last_position.map_or(window::Position::Default, |last_position| {
                                window::Position::Specific(last_position + Vector::new(20.0, 20.0))
                            });

                        let (_id, open) = window::open(window::Settings {
                            position,
                            size: Size::new(800.0, 400.0),
                            ..window::Settings::default()
                        });

                        open
                    })
                    .map(Message::WindowOpened)
            }
            Message::WindowOpened(id) => {
                let focus_input = text_input::focus(format!("input-{id}"));

                self.windows.insert(id, PlotWindow::new());

                focus_input
            }
            Message::WindowClosed(id) => {
                self.windows.remove(&id);

                if self.windows.is_empty() {
                    iced::exit()
                } else {
                    Task::none()
                }
            }
        }
    }

    pub fn view(&self, window_id: window::Id) -> Element<Message> {
        if let Some(window) = self.windows.get(&window_id) {
            center(window.view(window_id)).into()
        } else {
            horizontal_space().into()
        }
        Subscription::none()
    }

    pub fn subscription(&mut self) -> Subscription<Message> {
        // Subscription for window events
        let window_events = window::events().map(|event| {
            match event.1 {
                Event::Opened { position, size } => Message::WindowOpened(event.0),
                _ => {
                    // Handle other event types if necessary
                    // For now, we can ignore them
                    Message::None
                }
            }
        });

        // Subscription for command channel if available
        let command_subscription = if let Some(command_rx) = self.command_rx.take() {
            subscription::unfold(command_rx, |mut rx| async move {
                match rx.next().await {
                    Some(cmd) => (Some(Message::ExternalCommand(cmd)), rx),
                    None => (None, rx),
                }
            })
        } else {
            Subscription::none()
        };

        // Combine both subscriptions
        Subscription::batch(vec![window_events, command_subscription])
    }

    pub fn title(&self, window: Id) -> String {
        format!("NADIR Plot ({window})")
    }
}

#[derive(Debug)]
struct PlotWindow {}
impl PlotWindow {
    fn new() -> Self {
        Self {}
    }

    fn view(&self, id: window::Id) -> Element<Message> {
        column![button("new window").on_press(Message::OpenWindow)].into()
    }
}
