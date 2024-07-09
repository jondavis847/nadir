//#![windows_subsystem = "windows"]
//#![warn(missing_docs)]

use iced::{
    advanced::graphics::core::window::icon,
    alignment, font, keyboard,
    mouse::ScrollDelta,
    time::Instant,
    widget::{button, canvas::Canvas, container, text, text_input, Column, Row},
    window, Application, Command, Element, Length, Point, Settings, Size, Subscription,
};
use iced_aw::{card, modal};
use std::{env, path::Path};

mod app_state;
mod multibody_ui;
mod ui;

use app_state::AppState;
use multibody_ui::{BodyField, RevoluteField};

use ui::{
    dummies::{DummyBase, DummyBody, DummyComponent, DummyRevolute},
    errors::Errors,
    sim_tab::canvas::GraphCanvas,
    tab_bar::AppTabs,
    theme::Theme,
};

fn main() -> iced::Result {
    match env::current_dir() {
        Ok(path) => println!("The current working directory is: {}", path.display()),
        Err(e) => println!("Error getting current directory: {}", e),
    }

    let icon_path = Path::new("./resources/icon.png");
    dbg!(icon_path);
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
    IcedTest::run(settings)
}

// Define the possible user interactions
#[derive(Debug, Clone)]
enum Message {
    AnimationTick(Instant),
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
    ResultSelected(String),
    RevoluteConstantForceInputChanged(String),
    RevolutedampingInputChanged(String),
    RevoluteOmegaInputChanged(String),
    RevoluteNameInputChanged(String),
    RevoluteSpringConstantInputChanged(String),
    RevoluteThetaInputChanged(String),
    SimDtChanged(String),
    SimNameChanged(String),
    SimStartTimeChanged(String),
    SimStopTimeChanged(String),
    Simulate,
    PlotSimSelected(String),
    PlotComponentSelected(String),
    TabAnimationPressed,
    TabPlotPressed,
    TabSimulationPressed,
    LeftButtonPressed(Point),
    LeftButtonReleased(Point),
    MiddleButtonPressed(Point),
    RightButtonPressed(Point),
    RightButtonReleased(Point),
    CursorMoved(Point),
    WheelScrolled(ScrollDelta),
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
    type Theme = Theme;
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
        String::from("GADGT")
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
                Message::AnimationTick(instant) => state.animation(instant),
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
                Message::WheelScrolled(delta) => state.wheel_scrolled(delta),
                Message::DeletePressed => state.delete_pressed(),
                Message::EnterPressed => state.enter_pressed(),
                Message::TabPressed => state.tab_pressed(),
                Message::SaveComponent => state.save_component(),
                Message::WindowResized(size) => state.window_resized(size),
                Message::SimDtChanged(string) => state.simdiv.dt_changed(string),
                Message::SimStartTimeChanged(string) => state.simdiv.start_time_changed(string),
                Message::PlotSimSelected(string) => state.sim_selected(string),
                Message::PlotComponentSelected(string) => state.plot_component_selected(string),
                Message::SimStopTimeChanged(string) => state.simdiv.stop_time_changed(string),
                Message::SimNameChanged(string) => state.simdiv.name_changed(string),
                Message::Simulate => state.simulate(),
                Message::TabAnimationPressed => {
                    state.tab_bar.state.current_tab = AppTabs::Animation;
                    Command::none()
                }
                Message::TabPlotPressed => {
                    state.tab_bar.state.current_tab = AppTabs::Plot;
                    Command::none()
                }
                Message::TabSimulationPressed => {
                    state.tab_bar.state.current_tab = AppTabs::Simulation;
                    Command::none()
                }
                Message::ResultSelected(result) => {
                    dbg!(result);
                    Command::none()
                }
            },
        }
    }

    fn view(&self) -> Element<Message, Theme> {
        match self {
            IcedTest::Loading => loading_view(),
            IcedTest::Loaded(state) => loaded_view(state),
        }
    }

    fn subscription(&self) -> Subscription<Self::Message> {
        Subscription::batch(vec![
            window::frames().map(Message::AnimationTick),
            iced::event::listen_with(|event, _| match event {
                iced::Event::Window(_, window::Event::Resized { width, height }) => Some(
                    Message::WindowResized(Size::new(width as f32, height as f32)),
                ),
                iced::Event::Keyboard(keyboard::Event::KeyPressed { key, .. }) => match key {
                    keyboard::Key::Named(keyboard::key::Named::Enter) => {
                        Some(Message::EnterPressed)
                    }
                    keyboard::Key::Named(keyboard::key::Named::Delete) => {
                        Some(Message::DeletePressed)
                    }
                    keyboard::Key::Named(keyboard::key::Named::Tab) => Some(Message::TabPressed),
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
fn loaded_view(state: &AppState) -> Element<Message, Theme> {
    let tab_bar = state.tab_bar.content();

    let underlay: Element<Message, Theme> = match state.tab_bar.state.current_tab {
        AppTabs::Simulation => {
            let sim_div = container(state.simdiv.content())
                .width(Length::FillPortion(1))
                .height(Length::Fill);

            let graph_canvas = GraphCanvas::new(state);
            let graph_container = container(
                Canvas::new(graph_canvas)
                    .width(Length::Fill)
                    .height(Length::Fill),
            )
            .width(Length::FillPortion(4))
            .height(Length::Fill);

            Row::new()
                .push(sim_div)
                .push(graph_container)
                .height(Length::FillPortion(17))
                .width(Length::Fill)
                .into()
        }
        AppTabs::Plot => state.plot_tab.content().into(),
        _ => Row::new().into(), //nothing for now
    };

    let underlay = Column::new().push(tab_bar).push(underlay);

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

fn create_base_modal(_base: &DummyBase) -> Element<'static, Message, Theme> {
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

    //title doesnt work yet
    card("Base Information", content)
        .foot(footer)
        .max_width(500.0)
        .into()
}

fn create_body_modal(body: &DummyBody) -> Element<Message, Theme> {
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

fn create_revolute_modal(joint: &DummyRevolute) -> Element<Message, Theme> {
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

fn create_error_modal(error: Errors) -> Element<'static, Message, Theme> {
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
