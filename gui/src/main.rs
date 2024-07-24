//#![windows_subsystem = "windows"]
//#![warn(missing_docs)]

use iced::{
    advanced::graphics::core::window::icon,
    alignment, font, keyboard,
    mouse::ScrollDelta,
    time::Instant,
    widget::{canvas::Canvas, container, text, Column, Row},
    window, Application, Command, Element, Length, Point, Settings, Size, Subscription, Vector,
};
use iced_aw::modal;
use std::{env, path::Path};

mod app_state;
mod multibody_ui;
mod ui;

use app_state::AppState;
use multibody_ui::{BodyField, CuboidField, PrismaticField, RevoluteField};

use ui::{
    dummies::{DummyComponent, GeometryPickList, TransformPickList},
    modals::{
        create_base_modal, create_body_modal, create_error_modal, create_prismatic_modal,
        create_revolute_modal,
    },
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
    GadgtGui::run(settings)
}

// Define the possible user interactions
#[derive(Debug, Clone)]
enum Message {
    AnimationSimSelected(String),
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
    CuboidLengthInputChanged(String),
    CuboidWidthInputChanged(String),
    CuboidHeightInputChanged(String),
    GeometrySelected(GeometryPickList),
    ResultSelected(String),
    PrismaticConstantForceInputChanged(String),
    PrismaticDampingInputChanged(String),
    PrismaticVelocityInputChanged(String),
    PrismaticNameInputChanged(String),
    PrismaticSpringConstantInputChanged(String),
    PrismaticPositionInputChanged(String),
    RevoluteConstantForceInputChanged(String),
    RevoluteDampingInputChanged(String),
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
    PlotStateSelected(String),
    PlotComponentSelected(String),
    TabAnimationPressed,
    TabAnimationCameraRotation(Vector),
    TabPlotPressed,
    TabSimulationPressed,
    TransformSelected(TransformPickList),    
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
enum GadgtGui {
    Loading,
    Loaded(AppState),
}

async fn load() -> Result<(), String> {
    Ok(())
}

impl Application for GadgtGui {
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
            GadgtGui::Loading => {
                if let Message::Loaded(_) = message {
                    *self = GadgtGui::Loaded(AppState::default());
                }
                Command::none()
            }
            GadgtGui::Loaded(state) => match message {
                Message::AnimationSimSelected(string) => state.animation_sim_selected(string),
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
                Message::CuboidLengthInputChanged(value) => {
                    state.update_cuboid_field(CuboidField::Length, &value)
                }
                Message::CuboidWidthInputChanged(value) => {
                    state.update_cuboid_field(CuboidField::Width, &value)
                }
                Message::CuboidHeightInputChanged(value) => {
                    state.update_cuboid_field(CuboidField::Height, &value)
                }
                Message::GeometrySelected(geometry) => state.geometry_selected(geometry),
                Message::RevoluteConstantForceInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::ConstantForce, value)
                }
                Message::RevoluteDampingInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::Damping, value)
                }
                Message::RevoluteNameInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::Name, value)
                }
                Message::RevoluteOmegaInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::Omega, value)
                }
                Message::RevoluteSpringConstantInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::SpringConstant, value)
                }
                Message::RevoluteThetaInputChanged(value) => {
                    state.update_revolute_field(RevoluteField::Theta, value)
                }

                Message::PrismaticConstantForceInputChanged(value) => {
                    state.update_prismatic_field(PrismaticField::ConstantForce, value)
                }
                Message::PrismaticDampingInputChanged(value) => {
                    state.update_prismatic_field(PrismaticField::Damping, value)
                }
                Message::PrismaticNameInputChanged(value) => {
                    state.update_prismatic_field(PrismaticField::Name, value)
                }
                Message::PrismaticVelocityInputChanged(value) => {
                    state.update_prismatic_field(PrismaticField::Velocity, value)
                }
                Message::PrismaticSpringConstantInputChanged(value) => {
                    state.update_prismatic_field(PrismaticField::SpringConstant, value)
                }
                Message::PrismaticPositionInputChanged(value) => {
                    state.update_prismatic_field(PrismaticField::Position, value)
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
                Message::PlotSimSelected(string) => state.plot_sim_selected(string),
                Message::PlotStateSelected(string) => state.plot_state_selected(string),
                Message::PlotComponentSelected(string) => state.plot_component_selected(string),
                Message::SimStopTimeChanged(string) => state.simdiv.stop_time_changed(string),
                Message::SimNameChanged(string) => state.simdiv.name_changed(string),
                Message::Simulate => state.simulate(),
                Message::TabAnimationPressed => {
                    state.tab_bar.state.current_tab = AppTabs::Animation;
                    Command::none()
                }
                Message::TabAnimationCameraRotation(delta) => {
                    state
                        .animation_tab
                        .scene
                        .camera
                        .update_position_from_mouse_delta(delta);
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
                Message::ResultSelected(_) => Command::none(),
                Message::TransformSelected(transform) => state.transform_selected(transform),
            },
        }
    }

    fn view(&self) -> Element<Message, Theme> {
        match self {
            GadgtGui::Loading => loading_view(),
            GadgtGui::Loaded(state) => loaded_view(state),
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
                .height(Length::FillPortion(15))
                .width(Length::Fill)
                .into()
        }
        AppTabs::Plot => state.plot_tab.content().into(),
        AppTabs::Animation => state.animation_tab.content().into(),
    };

    let underlay = Column::new().push(tab_bar).push(underlay);

    let overlay = if let Some(active_error) = state.active_error {
        Some(create_error_modal(active_error))
    } else if let Some(active_modal) = state.modal {
        match active_modal.dummy_type {
            DummyComponent::Base => Some(create_base_modal(&state.nodebar.dummies.base)),
            DummyComponent::Body => Some(create_body_modal(
                &state.nodebar.dummies.body,
                &state.nodebar.dummies.cuboid,
            )),
            DummyComponent::Revolute => {
                Some(create_revolute_modal(&state.nodebar.dummies.revolute))
            }
            DummyComponent::Prismatic => {
                Some(create_prismatic_modal(&state.nodebar.dummies.prismatic))
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
