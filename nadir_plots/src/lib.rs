use iced::widget::{button, center, column, horizontal_space, text_input};
use iced::window::{self, Id};
use iced::{Element, Size, Subscription, Task, Vector};

use std::collections::HashMap;

// Define the plot manager commands
#[derive(Debug, Clone)]
pub enum PlotCommand {
    NewFigure,
}

#[derive(Default)]
pub struct PlotManager {
    windows: HashMap<window::Id, PlotWindow>,
}

#[derive(Debug, Clone)]
pub enum Message {
    ExternalCommand(PlotCommand),
    OpenWindow,
    WindowOpened(window::Id),
    WindowClosed(window::Id),
}

impl PlotManager {
    pub fn new() -> (Self, Task<Message>) {
        let (_id, open) = window::open(window::Settings::default());

        (
            Self {
                windows: HashMap::new(),
            },
            open.map(Message::WindowOpened),
        )
    }

    pub fn update(&mut self, message: Message) -> Task<Message> {
        match message {
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
    }

    pub fn subscription(
        state: &Self,
        command_rx: Arc<Mutex<Receiver<PlotCommand>>>,
    ) -> iced::Subscription<Message> {
        // Combine subscriptions: window events and command channel
        subscription::Subscription::batch(
            // 1. Listen for window events from all plots
            state
                .plots
                .values()
                .map(|plot| {
                    window::events(plot.id).map(move |event| Message::WindowEvent(plot.id, event))
                })
                .chain(
                    // 2. Listen for commands from the REPL
                    std::iter::once(subscription::unfold(
                        "command_channel",
                        command_rx,
                        |rx| async move {
                            // Try to get the mutex lock
                            if let Ok(rx_guard) = rx.lock() {
                                // Try to receive a command
                                match rx_guard.recv() {
                                    Ok(cmd) => {
                                        // We got a command from the REPL
                                        (Some(Message::ExternalCommand(cmd)), rx)
                                    }
                                    Err(_) => {
                                        // Channel closed, but keep subscription alive
                                        (None, rx)
                                    }
                                }
                            } else {
                                // Couldn't get lock, try again later
                                (None, rx)
                            }
                        },
                    )),
                )
                .collect(),
        )
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
