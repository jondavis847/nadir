//#![windows_subsystem = "windows"]
//#![warn(missing_docs)]

use iced::{
    alignment, font, keyboard,
    mouse::Cursor,
    widget::{
        button,
        canvas::{Cache, Canvas},
        container, text, text_input, Column, Row,
    },
    window, Application, Command, Element, Length, Settings, Size, Subscription,
};

use iced_aw::{card, modal};
use multibody::joint::Joint;
use std::{
    ops::Mul,
    time::{Duration, Instant},
};

mod multibody_ui;
mod ui;
use multibody_ui::{BodyField, MultibodyComponent, RevoluteField};
use ui::canvas::graph::{Graph, GraphMessage};
use ui::canvas::nodebar::{Nodebar, NodebarMessage};
use ui::canvas::GraphCanvas;
use ui::dummies::{DummyBase, DummyBody, DummyComponent, DummyRevolute, DummyTrait};
use ui::errors::Errors;
use ui::modals::ActiveModal;

fn main() -> iced::Result {
    let mut settings = Settings::default();
    settings.antialiasing = true;
    IcedTest::run(settings)
}

// Define the possible user interactions
#[derive(Debug, Clone)]
enum Message {
    BodyNameInputChanged(String),
    BodyMassInputChanged(String),
    BodyCmxInputChanged(String),
    BodyCmyInputChanged(String),
    BodyCmzInputChanged(String),
    BodyIxxInputChanged(String),
    BodyIyyInputChanged(String),
    BodyIzzInputChanged(String),
    BodyIxyInputChanged(String),
    BodyIxzInputChanged(String),
    BodyIyzInputChanged(String),
    RevoluteConstantForceInputChanged(String),
    RevolutedampingInputChanged(String),
    RevoluteOmegaInputChanged(String),
    RevoluteNameInputChanged(String),
    RevoluteSpringConstantInputChanged(String),
    RevoluteThetaInputChanged(String),
    LeftButtonPressed(Cursor),
    LeftButtonReleased(Cursor),
    MiddleButtonPressed(Cursor),
    RightButtonPressed(Cursor),
    RightButtonReleased(Cursor),
    CursorMoved(Cursor),
    CloseError,
    CloseModal,
    DeletePressed,
    EnterPressed,
    TabPressed,
    FontLoaded(Result<(), font::Error>),
    Loaded(Result<(), String>),
    SaveComponent,
    WindowResized(Size),
}

#[derive(Debug)]
enum IcedTest {
    Loading,
    Loaded(AppState),
}

#[derive(Debug)]
struct AppState {
    active_error: Option<Errors>,
    cache: Cache,
    counter_body: usize,
    counter_revolute: usize,
    graph: Graph,
    left_clicked_time_1: Option<Instant>,
    left_clicked_time_2: Option<Instant>,
    modal: Option<ActiveModal>,
    nodebar: Nodebar,
    theme: crate::ui::theme::Theme,
}

impl Default for AppState {
    fn default() -> Self {
        Self {
            active_error: None,
            cache: Cache::new(),
            counter_body: 0,
            counter_revolute: 0,
            left_clicked_time_1: None,
            left_clicked_time_2: None,
            graph: Graph::default(),
            modal: None,
            nodebar: Nodebar::default(),
            theme: crate::ui::theme::Theme::ORANGE,
        }
    }
}

enum MouseButton {
    Left,
    Right,
    Middle,
}

#[derive(Debug)]
enum MouseButtonReleaseEvents {
    SingleClick,
    DoubleClick,
    Held,
    Nothing,
}

impl AppState {
    pub fn close_error(&mut self) -> Command<Message> {
        self.active_error = None;
        Command::none()
    }

    pub fn close_modal(&mut self) -> Command<Message> {
        self.modal = None;
        Command::none()
    }

    pub fn cursor_moved(&mut self, cursor: Cursor) -> Command<Message> {
        let nodebar_redraw = self.nodebar.cursor_moved(cursor);
        let graph_redraw = self.graph.cursor_moved(cursor);

        // don't need to redraw just because mouse is moving
        if nodebar_redraw || graph_redraw {
            self.cache.clear();
        }
        Command::none()
    }

    pub fn delete_pressed(&mut self) -> Command<Message> {
        self.graph.delete_pressed();
        //self.nodebar.delete_pressed(); // no need for this, maybe ever?
        self.cache.clear();
        Command::none()
    }

    pub fn enter_pressed(&mut self) -> Command<Message> {
        // if the error modal is currently open, close it
        if self.active_error.is_some() {
            self.active_error = None;
        }
        //if a component modal is currently open, save it
        self.save_component()
    }

    pub fn left_button_pressed(&mut self, cursor: Cursor) -> Command<Message> {
        self.left_clicked_time_1 = self.left_clicked_time_2;
        self.left_clicked_time_2 = Some(Instant::now());

        self.nodebar.left_button_pressed(cursor);
        self.graph.left_button_pressed(cursor);
        self.cache.clear();
        Command::none()
    }

    pub fn left_button_released(&mut self, cursor: Cursor) -> Command<Message> {
        // Determine the type of mouse button release event
        let release_event = match (self.left_clicked_time_1, self.left_clicked_time_2) {
            (Some(clicked_time_1), Some(clicked_time_2)) => {
                let clicked_elapsed_time_1 = clicked_time_1.elapsed();
                let clicked_elapsed_time_2 = clicked_time_2.elapsed();

                if clicked_elapsed_time_1 <= Duration::from_millis(500) {
                    MouseButtonReleaseEvents::DoubleClick
                } else if clicked_elapsed_time_2 <= Duration::from_millis(300) {
                    MouseButtonReleaseEvents::SingleClick
                } else {
                    MouseButtonReleaseEvents::Held
                }
            }
            (None, Some(clicked_time_2)) => {
                if clicked_time_2.elapsed() <= Duration::from_millis(200) {
                    MouseButtonReleaseEvents::SingleClick
                } else {
                    MouseButtonReleaseEvents::Held
                }
            }
            _ => MouseButtonReleaseEvents::Nothing,
        };
        if let Some(NodebarMessage::NewComponent(id)) =
            self.nodebar.left_button_released(&release_event)
        {
            // Only create a new component if the mouse is over the graph
            if cursor.is_over(self.graph.bounds) {
                if self.nodebar.components.contains_key(&id) {
                    if let Some(component) = self.nodebar.components.get(&id) {
                        // Don't allow more than one base
                        let mut set_modal = false;

                        match component {
                            &DummyComponent::Base(_) => {
                                if self.graph.system.base.is_some() {
                                    self.active_error = Some(Errors::TooManyBases);
                                } else {
                                    set_modal = true;
                                }
                            }
                            _ => {
                                // Not a base, all good
                                set_modal = true;
                            }
                        }

                        if set_modal {
                            self.modal = Some(ActiveModal::new(id, None));
                        }
                    }
                }
            }
        }
        if let Some(GraphMessage::EditComponent((component_type, component_id))) =
            self.graph.left_button_released(&release_event, cursor)
        {
            match component_type {
                MultibodyComponent::Base => {
                    let base = &self.graph.system.base.unwrap();
                    let dummy = self
                        .nodebar
                        .components
                        .get_mut(&self.nodebar.map.base)
                        .unwrap();
                    match dummy {
                        DummyComponent::Base(dummy_base) => {
                            dummy_base.get_values_from(base);
                        }
                        _ => {} //TODO error, should not be possible
                    }
                }
                MultibodyComponent::Body => {
                    let body = self.graph.system.bodies.get(&component_id).unwrap();
                    let dummy = self
                        .nodebar
                        .components
                        .get_mut(&self.nodebar.map.body)
                        .unwrap();
                    match dummy {
                        DummyComponent::Body(dummy_body) => {
                            dummy_body.get_values_from(body);
                        }
                        _ => {} //TODO error, should not be possible
                    }
                }
                MultibodyComponent::Joint => {
                    let joint = self.graph.system.joints.get(&component_id).unwrap();
                    match joint {
                        Joint::Revolute(revolute) => {
                            let dummy = self
                                .nodebar
                                .components
                                .get_mut(&self.nodebar.map.revolute)
                                .unwrap();
                            match dummy {
                                DummyComponent::Revolute(dummy_rev) => {
                                    dummy_rev.get_values_from(revolute);
                                }
                                _ => {} //TODO error, should not be possible
                            }
                        }
                    }
                }
            }
            self.modal = Some(ActiveModal(component_type));
        }

        self.cache.clear();
        Command::none()
    }

    pub fn middle_button_pressed(&mut self, _cursor: Cursor) -> Command<Message> {
        //match self.graph.create_multibody_system() {
        //    Ok(system) => dbg!(system),
        //    Err(error) => {
                // TODO: handle error
        //        return Command::none();
        //    }
       // };
        Command::none()
    }

    pub fn right_button_pressed(&mut self, cursor: Cursor) -> Command<Message> {
        self.nodebar.right_button_pressed(cursor);
        self.graph.right_button_pressed(cursor);
        Command::none()
    }

    pub fn right_button_released(&mut self, cursor: Cursor) -> Command<Message> {
        self.graph.right_button_released(cursor);
        self.nodebar.right_button_released(cursor);
        self.cache.clear();
        Command::none()
    }

    pub fn save_component(&mut self) -> Command<Message> {
        // early return
        let modal = match self.modal {
            Some(ref modal) => modal,
            None => return Command::none(),
        };

        // early return
        match modal.0 {
            MultibodyComponent::Base => {
                if self.nodebar.dummies.base.name.is_empty() {
                    self.nodebar.dummies.base.name = "base".to_string();
                }
                let base = self.nodebar.dummies.base.to_base();
                self.graph.system.base = Some(base);
            },
            MultibodyComponent::Body => {
                if self.nodebar.dummies.body.name.is_empty() {
                    self.counter_body += 1;
                    self.nodebar.dummies.body.name = format!("body{}",self.counter_body).to_string();
                }
                let base = self.nodebar.dummies.base.to_base();
                self.graph.system.base = Some(base);
            }
        }
        

        // Ensure the body has a unique name if it's empty
        if dummy_component.get_name().is_empty() {
            let name = match dummy_component {
                DummyComponent::Base(_) => "base".to_string(),
                DummyComponent::Body(_) => {
                    self.counter_body += 1;
                    format!("body{}", self.counter_body)
                }
                DummyComponent::Revolute(_) => {
                    self.counter_revolute += 1;
                    format!("revolute{}", self.counter_revolute)
                }
            };
            dummy_component.set_name(&name);
        }
        match modal.graph_component_id {
            Some(id) => self.graph.edit_component(&dummy_component, id),
            None => self.graph.save_component(&dummy_component),
        };

        //TODO: actually do something with the error/message
        //match graph_message {

        //}

        // Clear the modal and cache
        dummy_component.clear();
        self.modal = None;
        self.cache.clear();
        Command::none()
    }

    fn tab_pressed(&mut self) -> Command<Message> {
        if self.modal.is_some() {
            Command::none()
            //iced::widget::focus_next() // not working right now
        } else {
            Command::none()
        }
    }

    pub fn update_body_field(&mut self, field: BodyField, value: &str) -> Command<Message> {
        if let Some(dummy_component) = self.nodebar.components.get_mut(&self.nodebar.map.body) {
            if let DummyComponent::Body(dummy_body) = dummy_component {
                match field {
                    BodyField::Name => dummy_body.set_name(value),
                    BodyField::Mass => dummy_body.mass = value.to_string(),
                    BodyField::Cmx => dummy_body.cmx = value.to_string(),
                    BodyField::Cmy => dummy_body.cmy = value.to_string(),
                    BodyField::Cmz => dummy_body.cmz = value.to_string(),
                    BodyField::Ixx => dummy_body.ixx = value.to_string(),
                    BodyField::Iyy => dummy_body.iyy = value.to_string(),
                    BodyField::Izz => dummy_body.izz = value.to_string(),
                    BodyField::Ixy => dummy_body.ixy = value.to_string(),
                    BodyField::Ixz => dummy_body.ixz = value.to_string(),
                    BodyField::Iyz => dummy_body.iyz = value.to_string(),
                }
            } else {
                // Handle error: must be the dummy body
                eprintln!("Error: Component is not a DummyBody");
            }
        }
        Command::none()
    }

    pub fn update_revolute_field(&mut self, field: RevoluteField, value: &str) -> Command<Message> {
        if let Some(dummy_component) = self.nodebar.components.get_mut(&self.nodebar.map.revolute) {
            if let DummyComponent::Revolute(dummy_revolute) = dummy_component {
                match field {
                    RevoluteField::Name => dummy_revolute.set_name(value),
                    RevoluteField::ConstantForce => {
                        dummy_revolute.constant_force = value.to_string()
                    }
                    RevoluteField::damping => dummy_revolute.damping = value.to_string(),
                    RevoluteField::Omega => dummy_revolute.omega = value.to_string(),
                    RevoluteField::SpringConstant => {
                        dummy_revolute.spring_constant = value.to_string()
                    }
                    RevoluteField::Theta => dummy_revolute.theta = value.to_string(),
                }
            } else {
                // Handle error: must be the dummy revolute
                eprintln!("Error: Component is not a DummyRevolute");
            }
        }
        Command::none()
    }

    fn window_resized(&mut self, window_size: Size) -> Command<Message> {
        let graph_size = Size::new(
            window_size.width - self.nodebar.bounds.width,
            window_size.height,
        );
        self.graph.window_resized(graph_size);
        let nodebar_size = Size::new(self.nodebar.bounds.width, window_size.height);
        self.nodebar.window_resized(nodebar_size);
        self.cache.clear();
        Command::none()
    }
}

async fn load() -> Result<(), String> {
    Ok(())
}

impl Application for IcedTest {
    type Message = Message;
    type Theme = crate::ui::theme::Theme;
    type Executor = iced::executor::Default;
    type Flags = ();

    fn new(_flags: ()) -> (Self, Command<Self::Message>) {
        (
            Self::Loading,
            Command::<Message>::batch(vec![
                font::load(iced_aw::BOOTSTRAP_FONT_BYTES).map(Message::FontLoaded),
                Command::perform(load(), Message::Loaded),
            ]),
        )
    }

    fn title(&self) -> String {
        String::from("jds")
    }

    fn update(&mut self, message: Message) -> Command<Message> {
        match self {
            IcedTest::Loading => {
                if let Message::Loaded(_) = message {
                    *self = IcedTest::Loaded(AppState::default());
                }
                Command::none()
            }
            IcedTest::Loaded(state) => match message {
                Message::FontLoaded(_) => Command::none(),
                Message::Loaded(_) => Command::none(),
                Message::BodyNameInputChanged(value) => {
                    state.update_body_field(BodyField::Name, &value)
                }
                Message::BodyMassInputChanged(value) => {
                    state.update_body_field(BodyField::Mass, &value)
                }
                Message::BodyCmxInputChanged(value) => {
                    state.update_body_field(BodyField::Cmx, &value)
                }
                Message::BodyCmyInputChanged(value) => {
                    state.update_body_field(BodyField::Cmy, &value)
                }
                Message::BodyCmzInputChanged(value) => {
                    state.update_body_field(BodyField::Cmz, &value)
                }
                Message::BodyIxxInputChanged(value) => {
                    state.update_body_field(BodyField::Ixx, &value)
                }
                Message::BodyIyyInputChanged(value) => {
                    state.update_body_field(BodyField::Iyy, &value)
                }
                Message::BodyIzzInputChanged(value) => {
                    state.update_body_field(BodyField::Izz, &value)
                }
                Message::BodyIxyInputChanged(value) => {
                    state.update_body_field(BodyField::Ixy, &value)
                }
                Message::BodyIxzInputChanged(value) => {
                    state.update_body_field(BodyField::Ixz, &value)
                }
                Message::BodyIyzInputChanged(value) => {
                    state.update_body_field(BodyField::Iyz, &value)
                }
                Message::RevoluteConstantForceInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::ConstantForce, &value)
                }
                Message::RevolutedampingInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::damping, &value)
                }
                Message::RevoluteNameInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::Name, &value)
                }
                Message::RevoluteOmegaInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::Omega, &value)
                }
                Message::RevoluteSpringConstantInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::SpringConstant, &value)
                }
                Message::RevoluteThetaInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::Theta, &value)
                }
                Message::LeftButtonPressed(cursor) => state.left_button_pressed(cursor),
                Message::LeftButtonReleased(cursor) => state.left_button_released(cursor),
                Message::MiddleButtonPressed(cursor) => state.middle_button_pressed(cursor),
                Message::RightButtonPressed(cursor) => state.right_button_pressed(cursor),
                Message::RightButtonReleased(cursor) => state.right_button_released(cursor),
                Message::CloseError => state.close_error(),
                Message::CloseModal => state.close_modal(),
                Message::CursorMoved(cursor) => state.cursor_moved(cursor),
                Message::DeletePressed => state.delete_pressed(),
                Message::EnterPressed => state.enter_pressed(),
                Message::TabPressed => state.tab_pressed(),
                Message::SaveComponent => state.save_component(),
                Message::WindowResized(size) => state.window_resized(size),
            },
        }
    }

    fn view(&self) -> Element<Message, crate::ui::theme::Theme> {
        match self {
            IcedTest::Loading => loading_view(),
            IcedTest::Loaded(state) => loaded_view(state),
        }
    }

    fn subscription(&self) -> Subscription<Self::Message> {
        iced::event::listen_with(|event, _| match event {
            iced::Event::Window(_, window::Event::Resized { width, height }) => Some(
                Message::WindowResized(Size::new(width as f32, height as f32)),
            ),
            iced::Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => match key {
                keyboard::Key::Named(keyboard::key::Named::Enter) => Some(Message::EnterPressed),
                keyboard::Key::Named(keyboard::key::Named::Delete) => Some(Message::DeletePressed),
                keyboard::Key::Named(keyboard::key::Named::Tab) => Some(Message::TabPressed),
                _ => None,
            },
            _ => None,
        })
    }
}
// Helper function to create the loading view
fn loading_view() -> Element<'static, Message, crate::ui::theme::Theme> {
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
fn loaded_view(state: &AppState) -> Element<Message, crate::ui::theme::Theme> {
    let graph_canvas = GraphCanvas::new(state);
    let graph_container = container(
        Canvas::new(graph_canvas)
            .width(Length::Fill)
            .height(Length::Fill),
    )
    .width(Length::Fill)
    .height(Length::Fill);

    let underlay = Row::new().push(graph_container);

    let overlay = if let Some(active_error) = state.active_error {
        Some(create_error_modal(active_error))
    } else if let Some(active_modal) = state.modal {
        // and there's a DummyComponent for that ActiveModal
        if let Some(dummy) = state
            .nodebar
            .components
            .get(&active_modal.dummy_component_id)
        {
            match dummy {
                DummyComponent::Base(base) => Some(create_base_modal(base)),
                DummyComponent::Body(body) => Some(create_body_modal(body)),
                DummyComponent::Revolute(joint) => Some(create_revolute_modal(joint)),
            }
        } else {
            None
        }
    } else {
        None
    };

    modal(underlay, overlay)
        .on_esc(Message::CloseModal)
        .align_y(alignment::Vertical::Center)
        .into()
}

fn create_base_modal(_base: &DummyBase) -> Element<'static, Message, crate::ui::theme::Theme> {
    let content = Column::new();
    let footer = Row::new()
        .spacing(10)
        .padding(5)
        .width(Length::Fill)
        .push(
            button("Cancel")
                .width(Length::Fill)
                .on_press(Message::CloseModal),
        )
        .push(
            button("Ok")
                .width(Length::Fill)
                .on_press(Message::SaveComponent),
        );

    card("Base Information", content)
        .foot(footer)
        .max_width(500.0)
        .into()
}

fn create_body_modal(body: &DummyBody) -> Element<Message, crate::ui::theme::Theme> {
    let create_text_input = |label: &str, value: &str, on_input: fn(String) -> Message| {
        Row::new()
            .spacing(10)
            .push(text(label).width(Length::FillPortion(1)))
            .push(
                text_input(label, value)
                    .on_input(on_input)
                    .on_submit(Message::SaveComponent)
                    .width(Length::FillPortion(4)),
            )
            .width(Length::Fill)
    };

    let content = Column::new()
        .push(create_text_input(
            "name",
            &body.get_name(),
            Message::BodyNameInputChanged,
        ))
        .push(create_text_input(
            "mass",
            &body.mass,
            Message::BodyMassInputChanged,
        ))
        .push(create_text_input(
            "cmx",
            &body.cmx,
            Message::BodyCmxInputChanged,
        ))
        .push(create_text_input(
            "cmy",
            &body.cmy,
            Message::BodyCmyInputChanged,
        ))
        .push(create_text_input(
            "cmz",
            &body.cmz,
            Message::BodyCmzInputChanged,
        ))
        .push(create_text_input(
            "ixx",
            &body.ixx,
            Message::BodyIxxInputChanged,
        ))
        .push(create_text_input(
            "iyy",
            &body.iyy,
            Message::BodyIyyInputChanged,
        ))
        .push(create_text_input(
            "izz",
            &body.izz,
            Message::BodyIzzInputChanged,
        ))
        .push(create_text_input(
            "ixy",
            &body.ixy,
            Message::BodyIxyInputChanged,
        ))
        .push(create_text_input(
            "ixz",
            &body.ixz,
            Message::BodyIxzInputChanged,
        ))
        .push(create_text_input(
            "iyz",
            &body.iyz,
            Message::BodyIyzInputChanged,
        ));

    let footer = Row::new()
        .spacing(10)
        .padding(5)
        .width(Length::Fill)
        .push(
            button("Cancel")
                .width(Length::Fill)
                .on_press(Message::CloseModal),
        )
        .push(
            button("Ok")
                .width(Length::Fill)
                .on_press(Message::SaveComponent),
        );

    card("Body Information", content)
        .foot(footer)
        .max_width(500.0)
        .into()
}

fn create_revolute_modal(joint: &DummyRevolute) -> Element<Message, crate::ui::theme::Theme> {
    let create_text_input = |label: &str, value: &str, on_input: fn(String) -> Message| {
        Row::new()
            .spacing(10)
            .push(text(label).width(Length::FillPortion(1)))
            .push(
                text_input(label, value)
                    .on_input(on_input)
                    .on_submit(Message::SaveComponent)
                    .width(Length::FillPortion(2)),
            )
            .width(Length::Fill)
    };

    let content = Column::new()
        .push(create_text_input(
            "name",
            &joint.name,
            Message::RevoluteNameInputChanged,
        ))
        .push(create_text_input(
            "theta",
            &joint.theta,
            Message::RevoluteThetaInputChanged,
        ))
        .push(create_text_input(
            "omega",
            &joint.omega,
            Message::RevoluteOmegaInputChanged,
        ))
        .push(create_text_input(
            "constant force",
            &joint.constant_force,
            Message::RevoluteConstantForceInputChanged,
        ))
        .push(create_text_input(
            "damping",
            &joint.damping,
            Message::RevolutedampingInputChanged,
        ))
        .push(create_text_input(
            "spring constant",
            &joint.theta,
            Message::RevoluteSpringConstantInputChanged,
        ));

    let footer = Row::new()
        .spacing(10)
        .padding(5)
        .width(Length::Fill)
        .push(
            button("Cancel")
                .width(Length::Fill)
                .on_press(crate::Message::CloseModal),
        )
        .push(
            button("Ok")
                .width(Length::Fill)
                .on_press(crate::Message::SaveComponent),
        );

    card("Revolute Information", content)
        .foot(footer)
        .max_width(500.0)
        .into()
}

fn create_error_modal(error: Errors) -> Element<'static, Message, crate::ui::theme::Theme> {
    let text = text(error.get_error_message());
    let content = Column::new().push(text);
    let footer = Row::new().spacing(10).padding(5).width(Length::Fill).push(
        button("Ok")
            .width(Length::Fill)
            .on_press(Message::CloseError),
    );

    card("Error!", content)
        .foot(footer)
        .max_width(500.0)
        .style(ui::theme::Card::Error)
        .into()
}
