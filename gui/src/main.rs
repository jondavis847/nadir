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
use multibody::{joint::Joint, MultibodyTrait};
use std::time::{Duration, Instant};

mod multibody_ui;
mod ui;
use multibody_ui::{BodyField, RevoluteField};
use ui::canvas::graph::{Graph, GraphMessage};
use ui::canvas::nodebar::{Nodebar, NodebarMessage};
use ui::canvas::GraphCanvas;
use ui::dummies::{DummyBase, DummyBody, DummyComponent, DummyRevolute};
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
        if let Some(NodebarMessage::NewComponent(active_modal)) =
            self.nodebar.left_button_released(&release_event)
        {
            // Only create a new component if the mouse is over the graph
            if cursor.is_over(self.graph.bounds) {
                match active_modal.dummy_type {
                    DummyComponent::Base => {
                        if self.graph.system.base.is_some() {
                            self.active_error = Some(Errors::TooManyBases);
                        } else {
                            self.modal = Some(active_modal);
                        }
                    }
                    DummyComponent::Body => {
                        self.modal = Some(active_modal);
                    }
                    DummyComponent::Revolute => {
                        self.modal = Some(active_modal);
                    }                    
                }
            }
        }
        if let Some(GraphMessage::EditComponent((component_type, component_id))) =
            self.graph.left_button_released(&release_event, cursor)
        {
            let dummy_type;
            match component_type {
                DummyComponent::Base => {
                    let base = self.graph.system.base.as_ref().unwrap();
                    self.nodebar.dummies.base.get_values_from(base);
                    dummy_type = DummyComponent::Base;
                }
                DummyComponent::Body => {
                    let body = self.graph.system.bodies.get(&component_id).unwrap();
                    self.nodebar.dummies.body.get_values_from(body);
                    dummy_type = DummyComponent::Body;
                }
                DummyComponent::Revolute => {
                    let joint = self.graph.system.joints.get(&component_id).unwrap();
                    match joint {
                        Joint::Revolute(revolute) => {
                            self.nodebar.dummies.revolute.get_values_from(revolute);
                            dummy_type = DummyComponent::Revolute;
                        }
                    }
                }
            }
            self.modal = Some(ActiveModal::new(dummy_type, Some(component_id)));
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

        match modal.dummy_type {
            DummyComponent::Base => {
                if self.nodebar.dummies.base.name.is_empty() {
                    self.nodebar.dummies.base.name = "base".to_string();
                }
                match modal.component_id {
                    Some(_) => {
                        let base = self.graph.system.base.as_mut().unwrap();
                        //editing existing component
                        self.nodebar.dummies.base.set_values_for(base);
                    }
                    None => {
                        // saving a new base - hopefully error was caught somewhere that cant have too many bases
                        let base = self.nodebar.dummies.base.to_base();
                        let id = *base.get_id();
                        let label = base.get_name().to_string();
                        self.graph.system.add_base(base).unwrap();
                        self.graph.save_component(&modal.dummy_type, id, label).unwrap();
                    }
                }
                self.nodebar.dummies.base.clear();
            }
            DummyComponent::Body => {
                if self.nodebar.dummies.body.name.is_empty() {
                    self.counter_body += 1;
                    self.nodebar.dummies.body.name =
                        format!("body{}", self.counter_body).to_string();
                }
                match modal.component_id {
                    Some(id) => {
                        //editing existing body
                        let body = self.graph.system.bodies.get_mut(&id).unwrap();
                        self.nodebar.dummies.body.set_values_for(body);
                    }
                    None => {
                        //creating new body
                        let body = self.nodebar.dummies.body.to_body();
                        let id = *body.get_id();
                        let label = body.get_name().to_string();
                        self.graph.system.add_body(body).unwrap();
                        self.graph.save_component(&modal.dummy_type, id, label).unwrap();
                    }
                }
                self.nodebar.dummies.body.clear();
            }
            DummyComponent::Revolute => {
                if self.nodebar.dummies.revolute.name.is_empty() {
                    self.counter_revolute += 1;
                    self.nodebar.dummies.revolute.name =
                        format!("revolute{}", self.counter_revolute).to_string();
                }
                match modal.component_id {
                    Some(id) => {
                        //editing existing joint
                        let joint = self.graph.system.joints.get_mut(&id).unwrap();
                        match joint {
                            Joint::Revolute(revolute) => {
                                self.nodebar.dummies.revolute.set_values_for(revolute)
                            }
                        }
                    }
                    None => {
                        //create new joint
                        let joint = self.nodebar.dummies.revolute.to_joint();
                        let id = *joint.get_id();
                        let label = joint.get_name().to_string();
                        self.graph.system.add_joint(joint).unwrap();
                        self.graph.save_component(&modal.dummy_type, id, label).unwrap();
                    }
                }

                self.nodebar.dummies.revolute.clear();
            }
        }

        //TODO: actually do something with the error/message
        //match graph_message {

        //}

        // Clear the modal and cache
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
        let dummy_body = &mut self.nodebar.dummies.body;

        match field {
            BodyField::Name => dummy_body.name = value.to_string(),
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

        Command::none()
    }

    pub fn update_revolute_field(&mut self, field: RevoluteField, value: &str) -> Command<Message> {
        let dummy_revolute = &mut self.nodebar.dummies.revolute;
        match field {
            RevoluteField::Name => dummy_revolute.name = value.to_string(),
            RevoluteField::ConstantForce => dummy_revolute.constant_force = value.to_string(),
            RevoluteField::damping => dummy_revolute.damping = value.to_string(),
            RevoluteField::Omega => dummy_revolute.omega = value.to_string(),
            RevoluteField::SpringConstant => dummy_revolute.spring_constant = value.to_string(),
            RevoluteField::Theta => dummy_revolute.theta = value.to_string(),
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
        match active_modal.dummy_type {
            DummyComponent::Base => Some(create_base_modal(&state.nodebar.dummies.base)),
            DummyComponent::Body => Some(create_body_modal(&state.nodebar.dummies.body)),
            DummyComponent::Revolute => {
                Some(create_revolute_modal(&state.nodebar.dummies.revolute))
            }
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
            &body.name,
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
