use iced::{
    Element, Length, Point, Size, Subscription, Task, Vector,
    futures::{
        SinkExt, Stream, StreamExt,
        channel::mpsc::{self, Sender},
        executor::block_on,
    },
    keyboard,
    mouse::ScrollDelta,
    stream,
    time::Instant,
    widget::{canvas, center, column},
    window::{self, Id, icon},
};
use std::{
    collections::HashMap,
    path::PathBuf,
    sync::{Arc, Mutex},
};
use uuid::Uuid;

use crate::{
    DaemonToRepl, ReplToSubscription,
    animation::{AnimationMessage, AnimationProgram},
    plotting::{PlotMessage, PlotProgram, figure::Figure},
};

struct WindowManagerChannels {
    daemon_to_repl: Sender<DaemonToRepl>,
    //repl_to_daemon: Receiver<ReplToDaemon>,
    //daemon_to_subscription: Option<Sender<DaemonToSubscription>>,
}

pub struct WindowManager {
    windows: HashMap<Id, NadirWindow>,
    window_requests: Arc<Mutex<HashMap<Uuid, NadirWindowRequest>>>,
    channels: WindowManagerChannels,
    active_window: Option<Id>,
}

#[derive(Debug, Clone)]
pub enum Message {
    AnimationMessage(AnimationMessage),
    AnimationTick(Instant),
    CancelRequest(Uuid),
    CheckRequestReady(Uuid),
    ClearCache(Id),
    CloseAllFigures,
    CursorMoved(Point),
    EscapePressed(Id),
    LoadAnimation(Uuid, Arc<Mutex<PathBuf>>),
    MouseMiddlePressed(Point),
    MouseMiddleReleased(Point),
    NewAnimation(Arc<Mutex<PathBuf>>),
    NewFigure(Arc<Mutex<Figure>>),
    OpenWindow(Uuid, Size),
    PlotMessage(PlotMessage),
    ReplToSubscription(Sender<ReplToSubscription>),
    ReplClosed,
    RightButtonPressed(Id, Point),
    RightButtonReleased(Id, Point),
    WheelScrolled(Point, ScrollDelta),
    WindowOpened(Uuid, Id),
    WindowClosed(Id),
    WindowFocused(Id),
    WindowResized(Id, Size),
    WindowUnFocused(Id),
}

impl WindowManager {
    pub fn new(daemon_to_repl: Sender<DaemonToRepl>) -> (Self, Task<Message>) {
        (
            Self {
                windows: HashMap::new(),
                window_requests: Arc::new(Mutex::new(HashMap::new())),
                channels: WindowManagerChannels { daemon_to_repl },
                active_window: None,
            },
            Task::none(),
        )
    }

    pub fn update(&mut self, message: Message) -> Task<Message> {
        match message {
            Message::AnimationMessage(message) => {
                if let Some(id) = &self.active_window {
                    if let Some(window) = self.windows.get_mut(id) {
                        window.process_animation_message(message);
                    }
                }
                Task::none()
            }
            Message::AnimationTick(instant) => {
                if let Some(id) = &self.active_window {
                    if let Some(window) = self.windows.get_mut(id) {
                        window.animation_tick(&instant);
                    }
                }
                Task::none()
            }
            Message::CancelRequest(request_id) => {
                self.window_requests.lock().unwrap().remove(&request_id);
                Task::none()
            }
            Message::CheckRequestReady(request_id) => {
                // First just check status without removing
                let ready = {
                    let window_requests = self.window_requests.lock().unwrap();
                    if let Some(request) = window_requests.get(&request_id) {
                        request.id.is_some() && request.program.is_some()
                    } else {
                        false
                    }
                };

                if ready {
                    // Now we know it's ready, remove it and process
                    let mut window_requests = self.window_requests.lock().unwrap();
                    if let Some(mut window_request) = window_requests.remove(&request_id) {
                        // These unwraps are safe because we already checked above
                        let id = window_request.id.unwrap();
                        let program = window_request.program.take().unwrap();

                        let mut window = NadirWindow::new(id, program);
                        match &mut window.program {
                            NadirProgram::Animation(animation) => animation.set_window_id(id),
                            NadirProgram::Plot(plot) => plot.set_window_id(id),
                        }
                        self.windows.insert(id, window);
                    }
                }

                Task::none()
            }
            Message::ClearCache(id) => {
                if let Some(window) = self.windows.get_mut(&id) {
                    match &mut window.program {
                        NadirProgram::Animation(_) => {
                            unreachable!("animation program should not send clear cache message")
                        }
                        NadirProgram::Plot(plot) => plot.clear_cache(),
                    }
                }
                Task::none()
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
            Message::CursorMoved(point) => {
                if let Some(id) = &self.active_window {
                    if let Some(window) = self.windows.get_mut(id) {
                        window.cursor_moved(point);
                    }
                }
                Task::none()
            }
            Message::EscapePressed(id) => {
                if let Some(window) = self.windows.get_mut(&id) {
                    window.escape_pressed();
                }
                Task::none()
            }
            Message::LoadAnimation(request_id, result_path) => {
                match AnimationProgram::new(result_path) {
                    Ok(animation) => {
                        if let Some(request) =
                            self.window_requests.lock().unwrap().get_mut(&request_id)
                        {
                            request.program = Some(NadirProgram::Animation(animation));
                            Task::done(Message::CheckRequestReady(request_id))
                        } else {
                            Task::done(Message::CancelRequest(request_id)) //Todo maybe this doesnt make sense since we didnt find the request id?
                        }
                    }
                    Err(e) => {
                        eprintln!("{e}");
                        Task::done(Message::CancelRequest(request_id))
                    }
                }
            }
            Message::MouseMiddlePressed(point) => {
                if let Some(id) = &self.active_window {
                    if let Some(window) = self.windows.get_mut(id) {
                        window.mouse_middle_pressed(point);
                    }
                }
                Task::none()
            }
            Message::MouseMiddleReleased(point) => {
                if let Some(id) = &self.active_window {
                    if let Some(window) = self.windows.get_mut(id) {
                        window.mouse_middle_released(point);
                    }
                }
                Task::none()
            }
            Message::NewAnimation(result_path) => {
                let request_id = Uuid::new_v4();
                self.window_requests
                    .lock()
                    .unwrap()
                    .insert(request_id, NadirWindowRequest::default());

                Task::batch([
                    Task::done(Message::LoadAnimation(request_id, result_path)),
                    Task::done(Message::OpenWindow(request_id, Size::new(1280.0, 720.0))),
                ])
            }
            Message::NewFigure(plot) => {
                let request_id = Uuid::new_v4();
                let mut window_request = NadirWindowRequest::default();
                window_request.program = Some(NadirProgram::Plot(PlotProgram::new(plot)));
                self.window_requests
                    .lock()
                    .unwrap()
                    .insert(request_id, window_request);

                Task::done(Message::OpenWindow(request_id, Size::new(720.0, 480.0)))
            }
            Message::OpenWindow(request_id, size) => {
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
                    .then(move |position| {
                        const ICON_BYTES: &[u8] = include_bytes!("../resources/nadir.png");

                        let (icon_rgba, icon_width, icon_height) = {
                            let image = image::load_from_memory(ICON_BYTES)
                                .expect("Failed to load icon from memory")
                                .into_rgba8();

                            let (width, height) = image.dimensions();
                            (image.into_raw(), width, height)
                        };

                        let icon = icon::from_rgba(icon_rgba, icon_width, icon_height).unwrap();

                        let (_id, open) = window::open(window::Settings {
                            position,
                            size,
                            icon: Some(icon),
                            ..window::Settings::default()
                        });
                        open
                    })
                    .then(move |id| Task::done(Message::WindowOpened(request_id, id)))
            }
            Message::PlotMessage(message) => {
                if let Some(id) = &self.active_window {
                    if let Some(window) = self.windows.get_mut(id) {
                        window.process_plot_message(message);
                    }
                }
                Task::none()
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
            Message::RightButtonPressed(_id, _point) => Task::none(),
            Message::RightButtonReleased(_id, _point) => Task::none(),
            Message::WheelScrolled(point, scroll_delta) => {
                if let Some(active_id) = self.active_window {
                    if let Some(window) = self.windows.get_mut(&active_id) {
                        window.wheel_scolled(point, scroll_delta);
                    }
                }

                Task::none()
            }
            Message::WindowOpened(request_id, id) => {
                if let Some(request) = self.window_requests.lock().unwrap().get_mut(&request_id) {
                    request.id = Some(id);
                    self.active_window = Some(id);
                    Task::done(Message::CheckRequestReady(request_id))
                } else {
                    Task::done(Message::CancelRequest(request_id))
                }
            }
            Message::WindowClosed(id) => {
                self.windows.remove(&id);
                Task::none()
            }
            Message::WindowFocused(id) => {
                self.active_window = Some(id);
                Task::none()
            }
            Message::WindowResized(id, size) => {
                if let Some(window) = self.windows.get_mut(&id) {
                    match &mut window.program {
                        NadirProgram::Animation(animation) => animation.window_resized(size), // window events handled inside the animation program
                        NadirProgram::Plot(plot) => plot.window_resized(size),
                    }
                }
                Task::none()
            }
            Message::WindowUnFocused(id) => {
                if let Some(active_id) = self.active_window {
                    if active_id == id {
                        self.active_window = None;
                    }
                }
                Task::none()
            }
        }
    }

    pub fn view(&self, window_id: window::Id) -> Element<Message> {
        if let Some(window) = self.windows.get(&window_id) {
            center(window.view()).into()
        } else {
            // not sure why we get into this, view outpacing the storage into self.windows?
            column![].into()
        }
    }

    pub fn subscription(&self) -> Subscription<Message> {
        let iced_events = iced::event::listen_with(|event, _, id| match event {
            iced::Event::Window(window_event) => match window_event {
                window::Event::Closed => Some(Message::WindowClosed(id)),
                window::Event::Focused => Some(Message::WindowFocused(id)),
                window::Event::Unfocused => Some(Message::WindowUnFocused(id)),
                window::Event::Resized(size) => Some(Message::WindowResized(id, size)),
                _ => None,
            },
            iced::Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => match key {
                keyboard::Key::Named(keyboard::key::Named::Escape) => {
                    Some(Message::EscapePressed(id))
                }
                keyboard::Key::Named(keyboard::key::Named::Delete) => None,
                //keyboard::Key::Named(keyboard::key::Named::Tab) => Some(Message::TabPressed),
                _ => None,
            },
            _ => None,
        });

        let animation_tick = window::frames().map(Message::AnimationTick);
        let plot_commands = Subscription::run(plot_subscription);

        // Combine both subscriptions
        Subscription::batch(vec![iced_events, plot_commands, animation_tick])
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
                    ReplToSubscription::Animate(result_path) => {
                        output
                            .send(Message::NewAnimation(result_path))
                            .await
                            .expect("error sending Animate from subscription to daemon");
                    }
                    ReplToSubscription::ClearCache(id) => {
                        output
                            .send(Message::ClearCache(id))
                            .await
                            .expect("error sending ClearCache from subscription to daemon");
                    }
                    ReplToSubscription::CloseAllFigures => {
                        output
                            .send(Message::CloseAllFigures)
                            .await
                            .expect("error sending CloseAllFigures from subscription to daemon");
                    }
                    ReplToSubscription::NewFigure(plot) => {
                        output
                            .send(Message::NewFigure(plot))
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
enum NadirProgram {
    Animation(AnimationProgram),
    Plot(PlotProgram),
}

impl From<AnimationProgram> for NadirProgram {
    fn from(value: AnimationProgram) -> Self {
        NadirProgram::Animation(value)
    }
}

#[derive(Debug)]
pub struct NadirWindow {
    //id: Id,
    program: NadirProgram,
}

impl NadirWindow {
    fn animation_tick(&mut self, instant: &Instant) {
        match &mut self.program {
            NadirProgram::Animation(animation) => animation.tick(instant),
            NadirProgram::Plot(_) => {}
        }
    }

    fn cursor_moved(&mut self, point: Point) {
        match &mut self.program {
            NadirProgram::Animation(animation) => animation.cursor_moved(point),
            NadirProgram::Plot(plot) => plot.cursor_moved(point),
        }
    }

    fn escape_pressed(&mut self) {
        match &mut self.program {
            NadirProgram::Animation(animation) => animation.escape_pressed(),
            NadirProgram::Plot(_) => {}
        }
    }

    fn mouse_middle_pressed(&mut self, point: Point) {
        match &mut self.program {
            NadirProgram::Animation(_) => {}
            NadirProgram::Plot(plot) => plot.mouse_middle_clicked(point),
        }
    }
    fn mouse_middle_released(&mut self, point: Point) {
        match &mut self.program {
            NadirProgram::Animation(_) => {}
            NadirProgram::Plot(plot) => plot.mouse_middle_released(point),
        }
    }

    fn new(_id: Id, program: NadirProgram) -> Self {
        Self { program }
    }

    fn process_animation_message(&mut self, message: AnimationMessage) {
        match &mut self.program {
            NadirProgram::Animation(animation) => animation.update(message),
            NadirProgram::Plot(_) => unreachable!(
                "plots should not send animation messages, or the active window got messed up"
            ),
        }
    }

    fn process_plot_message(&mut self, message: PlotMessage) {
        match &mut self.program {
            NadirProgram::Animation(_) => unreachable!(
                "plots should not send animation messages, or the active window got messed up"
            ),
            NadirProgram::Plot(plot) => plot.update(message),
        }
    }

    fn view(&self) -> Element<Message> {
        match &self.program {
            NadirProgram::Animation(animation) => animation.content(),
            NadirProgram::Plot(plot) => {
                column![canvas(plot).height(Length::Fill).width(Length::Fill)].into()
            }
        }
    }

    fn wheel_scolled(&mut self, point: Point, scroll_delta: ScrollDelta) {
        match &mut self.program {
            NadirProgram::Animation(animation) => animation.wheel_scrolled(scroll_delta),
            NadirProgram::Plot(plot) => plot.wheel_scrolled(point, scroll_delta),
        }
    }
}

#[derive(Debug, Default)]
pub struct NadirWindowRequest {
    id: Option<Id>,
    program: Option<NadirProgram>,
}
