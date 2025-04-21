use std::{collections::HashMap, path::PathBuf};

use animation::animation::AnimationGui;
use iced::{
    Element, Size, Subscription, Task, Vector,
    futures::{
        SinkExt, Stream, StreamExt,
        channel::mpsc::{self, Sender},
        executor::block_on,
    },
    stream,
    widget::{button, center, column},
    window::{self, Event, icon},
};

use crate::{DaemonToRepl, ReplToSubscription};

struct PlotManagerChannels {
    daemon_to_repl: Sender<DaemonToRepl>,
    //repl_to_daemon: Receiver<ReplToDaemon>,
    //daemon_to_subscription: Option<Sender<DaemonToSubscription>>,
}

pub struct PlotManager {
    window_requests: Vec<WindowRequest>,
    windows: HashMap<window::Id, NadirWindow>,
    channels: PlotManagerChannels,
}

#[derive(Debug, Clone)]
pub enum Message {
    Animate(PathBuf),
    None,
    CloseAllFigures,
    Plot,
    ReplToSubscription(Sender<ReplToSubscription>),
    ReplClosed,
    NewFigure,
    OpenWindow,
    WindowOpened(window::Id),
    WindowClosed(window::Id),
}

impl PlotManager {
    pub fn new(daemon_to_repl: Sender<DaemonToRepl>) -> (Self, Task<Message>) {
        (
            Self {
                window_requests: Vec::with_capacity(5),
                windows: HashMap::new(),
                channels: PlotManagerChannels { daemon_to_repl },
            },
            Task::none(),
        )
    }

    pub fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::Animate(result_path) => {
                let (animation_gui, _) = AnimationGui::new(result_path);
                self.windows.insert()
                Task::perform(Message::OpenWindow)
            }
            Message::CloseAllFigures => {
                // If no windows are open, just return a no-op task
                if self.windows.is_empty() {
                    return Task::none();
                }

                // Create a batch of close commands for all windows
                let close_commands: Vec<Task<Message>> = self
                    .windows
                    .keys()
                    .map(|id| {
                        // Create a close command for each window
                        let window_id = *id;
                        window::close(window_id).map(move |_: ()| Message::WindowClosed(window_id))
                    })
                    .collect();

                // Return a batch that contains all the close commands
                // The Message::WindowClosed will be triggered for each window
                // which will update your state
                Task::batch(close_commands)
            }
            Message::None => return Task::none(),
            Message::NewFigure => {
                // Return a task that will emit the OpenWindow message
                Task::perform(async {}, |_| Message::OpenWindow(NadirWindows::Plot))
            }
            Message::ReplToSubscription(repl_to_subscription) => {
                block_on(
                    self.channels
                        .daemon_to_repl
                        .send(DaemonToRepl::ReplToSubscriptionTx(repl_to_subscription)),
                )
                .expect("error sending repl_to_subscription_tx from daemon to repl");

                Task::none()
            }
            Message::ReplClosed => iced::exit(),

            Message::OpenWindow => {
                // Create a task that will open a window
                let position = if let Some(last_window) = self.windows.keys().last() {
                    // If we have a previous window, position relative to it
                    window::get_position(*last_window).map(|maybe_position| {
                        maybe_position.map_or(window::Position::Default, |last_position| {
                            window::Position::Specific(last_position + Vector::new(20.0, 20.0))
                        })
                    })
                } else {
                    // No previous window, use default position
                    Task::perform(async {}, |_| window::Position::Default)
                };

                // Chain the window opening after we determine the position
                position
                    .then(|position| {
                        const ICON_BYTES: &[u8] = include_bytes!("../resources/nadir.png");

                        let (icon_rgba, icon_width, icon_height) = {
                            let image = image::load_from_memory(ICON_BYTES)
                                .expect("Failed to load icon from memory")
                                .into_rgba8();

                            let (width, height) = image.dimensions();
                            (image.into_raw(), width, height)
                        };

                        let icon = icon::from_rgba(icon_rgba, icon_width, icon_height).unwrap();

                        let (id, open) = window::open(window::Settings {
                            position,
                            size: Size::new(800.0, 400.0),
                            icon: Some(icon),
                            ..window::Settings::default()
                        });
                        open
                    })
                    .map(Message::WindowOpened)
            }
            Message::Plot => Task::none(),
            Message::WindowOpened(id) => {
                self.windows.insert(id, NadirWindow::new());
                Task::none()
            }
            Message::WindowClosed(id) => {
                self.windows.remove(&id);
                Task::none()
            }
        }
    }

    pub fn view(&self, window_id: window::Id) -> Element<Message> {
        if let Some(window) = self.windows.get(&window_id) {
            center(window.view(window_id)).into()
        } else {
            // not sure why we get into this, view outpacing the storage into self.windows?
            column![].into()
        }
    }

    pub fn subscription(&self) -> Subscription<Message> {
        // Subscription for window events
        let window_events = window::events().map(|event| {
            match event.1 {
                Event::Opened { .. } => Message::WindowOpened(event.0),
                Event::Closed => Message::WindowClosed(event.0),
                _ => {
                    // Handle other event types if necessary
                    // For now, we can ignore them
                    Message::None
                }
            }
        });

        let plot_commands = Subscription::run(plot_subscription);

        // Combine both subscriptions
        Subscription::batch(vec![window_events, plot_commands])
    }

    pub fn title(&self, id: window::Id) -> String {
        format!("NADIR Plot ({id})")
    }
}

fn plot_subscription() -> impl Stream<Item = Message> {
    stream::channel(100, |mut output| async move {
        // Create subscription -> daemon channel
        // let (subscription_to_daemon_tx, mut subscription_to_daemon_rx) =
        //     mpsc::channel::<SubscriptionToDaemon>(100);
        // Create repl -> subscription channel
        let (repl_to_subscription_tx, mut repl_to_subscription_rx) =
            mpsc::channel::<ReplToSubscription>(100);

        // Send the sender back to the application
        output
            .send(Message::ReplToSubscription(repl_to_subscription_tx))
            .await
            .expect("error sending subscription tx to repl");

        loop {
            // read from repl
            if let Some(input) = repl_to_subscription_rx.next().await {
                match input {
                    ReplToSubscription::Animate => {
                        output
                            .send(Message::Animate)
                            .await
                            .expect("error sending Animate from subscription to daemon");
                    }
                    ReplToSubscription::CloseAllFigures => {
                        output
                            .send(Message::CloseAllFigures)
                            .await
                            .expect("error sending CloseAllFigures from subscription to daemon");
                    }
                    ReplToSubscription::NewFigure => {
                        output
                            .send(Message::NewFigure)
                            .await
                            .expect("error sending NewFigure from subscription to daemon");
                    }
                    ReplToSubscription::ReplClosed => {
                        output
                            .send(Message::ReplClosed)
                            .await
                            .expect("error sending ReplClosed from subscription to daemon");
                        break;
                    }
                }
            }
        }
    })
}

#[derive(Debug)]
struct NadirWindow {
    id: window::Id,
    window_type: NadirWindows,
}
impl NadirWindow {
    fn new(id: window::Id, window_type: NadirWindows) -> Self {
        Self {id,window_type}
    }

    fn view(&self, _id: window::Id) -> Element<Message> {
        column![button("new window").on_press(Message::OpenWindow)].into()
    }
}

#[derive(Debug)]
enum NadirWindows {
    Animation(AnimationGui),
    Plot,
}

pub struct WindowRequest {
    window_type: NadirWindows,
}
