use std::collections::HashMap;

use iced::{
    Element, Size, Subscription, Task, Vector,
    futures::{
        SinkExt, Stream, StreamExt,
        channel::mpsc::{self, Receiver, Sender},
        executor::block_on,
    },
    stream,
    widget::{button, center, column, horizontal_space, text_input},
    window::{self, Event, Id},
};

use crate::{DaemonToRepl, DaemonToSubscription, ReplToDaemon, ReplToSubscription};

struct PlotManagerChannels {
    daemon_to_repl: Sender<DaemonToRepl>,
    repl_to_daemon: Receiver<ReplToDaemon>,
    daemon_to_subscription: Option<Sender<DaemonToSubscription>>,
}

pub struct PlotManager {
    windows: HashMap<window::Id, PlotWindow>,
    channels: PlotManagerChannels,
}

#[derive(Debug, Clone)]
pub enum Message {
    None,
    CloseAllFigures,
    ReplToSubscription(Sender<ReplToSubscription>),
    ReplClosed,
    NewFigure,
    OpenWindow,
    WindowOpened(window::Id),
    WindowClosed(window::Id),
}

impl PlotManager {
    pub fn new(
        daemon_to_repl: Sender<DaemonToRepl>,
        repl_to_daemon: Receiver<ReplToDaemon>,
    ) -> (Self, Task<Message>) {
        (
            Self {
                windows: HashMap::new(),
                channels: PlotManagerChannels {
                    daemon_to_repl,
                    repl_to_daemon,
                    daemon_to_subscription: None,
                },
            },
            Task::none(),
        )
    }

    pub fn update(&mut self, message: Message) -> Task<Message> {
        match message {
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
                Task::perform(async {}, |_| Message::OpenWindow)
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
                Task::none()
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

    pub fn subscription(&self) -> Subscription<Message> {
        // Subscription for window events
        let window_events = window::events().map(|event| {
            match event.1 {
                Event::Opened { position, size } => Message::WindowOpened(event.0),
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
struct PlotWindow {}
impl PlotWindow {
    fn new() -> Self {
        Self {}
    }

    fn view(&self, id: window::Id) -> Element<Message> {
        column![button("new window").on_press(Message::OpenWindow)].into()
    }
}
