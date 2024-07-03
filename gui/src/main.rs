//#![windows_subsystem = "windows"]
//#![warn(missing_docs)]

use iced::{
    alignment, font, keyboard,
    mouse::Cursor,
    widget::{
        button,
        canvas::Canvas,
        container, text, text_input, Column, Row,
    },
    window, Application, Command, Element, Length, Settings, Size, Subscription,
};

use iced_aw::{card, modal};
mod app_state;
mod multibody_ui;
mod ui;

use app_state::AppState;
use multibody_ui::{BodyField, RevoluteField};
use ui::canvas::GraphCanvas;
use ui::dummies::{DummyBase, DummyBody, DummyComponent, DummyRevolute};
use ui::errors::Errors;

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
                    state.update_revolute_field(RevoluteField::Damping, &value)
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
