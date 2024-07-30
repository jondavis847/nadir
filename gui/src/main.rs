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
use rotations::{axes::Axis, euler_angles::EulerSequence};
use std::{env, path::Path};

mod app_state;
mod multibody_ui;
mod ui;

use app_state::AppState;
use multibody_ui::{
    BodyField, CartesianField, CuboidField, CylindricalField, EulerAnglesField, PrismaticField,
    QuaternionField, RevoluteField, RotationMatrixField, SphericalField,
};

use ui::{
    dummies::{
        DummyComponent, GeometryPickList, GravityPickList, RotationPickList, TransformPickList,
        TranslationPickList, TwoBodyPickList,
    },
    modals::{
        create_base_modal, create_body_modal, create_error_modal, create_gravity_modal, create_prismatic_modal,
        create_revolute_modal, create_transform_modal,
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
    AxisPrimaryFromSelected(Axis),
    AxisPrimaryToSelected(Axis),
    AxisSecondaryFromSelected(Axis),
    AxisSecondaryToSelected(Axis),
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
    CartesianXChanged(String),
    CartesianYChanged(String),
    CartesianZChanged(String),
    CloseError,
    CloseModal,
    ConstantGravityXChanged(String),
    ConstantGravityYChanged(String),
    ConstantGravityZChanged(String),
    CuboidLengthInputChanged(String),
    CuboidWidthInputChanged(String),
    CuboidHeightInputChanged(String),
    CursorMoved(Point),
    CylindricalAzimuthChanged(String),
    CylindricalHeightChanged(String),
    CylindricalRadiusChanged(String),
    DeletePressed,
    EnterPressed,
    EulerAngleSequenceChanged(EulerSequence),
    EulerAnglePhiChanged(String),
    EulerAngleThetaChanged(String),
    EulerAnglePsiChanged(String),
    FontLoaded(Result<(), font::Error>),
    GeometrySelected(GeometryPickList),
    GravityModelSelected(GravityPickList),
    LeftButtonPressed(Point),
    LeftButtonReleased(Point),
    Loaded(Result<(), String>),
    MiddleButtonPressed(Point),
    PlotSimSelected(String),
    PlotStateSelected(String),
    PlotComponentSelected(String),
    PrismaticConstantForceInputChanged(String),
    PrismaticDampingInputChanged(String),
    PrismaticVelocityInputChanged(String),
    PrismaticNameInputChanged(String),
    PrismaticSpringConstantInputChanged(String),
    PrismaticPositionInputChanged(String),
    QuaternionWChanged(String),
    QuaternionXChanged(String),
    QuaternionYChanged(String),
    QuaternionZChanged(String),
    ResultSelected(String),
    RevoluteConstantForceInputChanged(String),
    RevoluteDampingInputChanged(String),
    RevoluteOmegaInputChanged(String),
    RevoluteNameInputChanged(String),
    RevoluteSpringConstantInputChanged(String),
    RevoluteThetaInputChanged(String),
    RightButtonPressed(Point),
    RightButtonReleased(Point),
    RotationMatrixE11Changed(String),
    RotationMatrixE12Changed(String),
    RotationMatrixE13Changed(String),
    RotationMatrixE21Changed(String),
    RotationMatrixE22Changed(String),
    RotationMatrixE23Changed(String),
    RotationMatrixE31Changed(String),
    RotationMatrixE32Changed(String),
    RotationMatrixE33Changed(String),
    SaveComponent,
    SimDtChanged(String),
    SimNameChanged(String),
    SimStartTimeChanged(String),
    SimStopTimeChanged(String),
    Simulate,
    SphericalAzimuthChanged(String),
    SphericalInclinationChanged(String),
    SphericalRadiusChanged(String),
    TabAnimationPressed,
    TabAnimationCameraRotation(Vector),
    TabPlotPressed,
    TabPressed,
    TabSimulationPressed,
    TransformSelected(TransformPickList),
    TransformRotationSelected(RotationPickList),
    TransformTranslationSelected(TranslationPickList),
    TwoBodyCustomMuChanged(String),
    TwoBodyModelSelected(TwoBodyPickList),
    WheelScrolled(ScrollDelta),
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
                Message::AxisPrimaryFromSelected(value) => state.axis_primary_from_selected(value),
                Message::AxisPrimaryToSelected(value) => state.axis_primary_to_selected(value),
                Message::AxisSecondaryFromSelected(value) => {
                    state.axis_secondary_from_selected(value)
                }
                Message::AxisSecondaryToSelected(value) => state.axis_secondary_to_selected(value),

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
                Message::CartesianXChanged(value) => {
                    state.update_cartesian_field(CartesianField::X, value)
                }
                Message::CartesianYChanged(value) => {
                    state.update_cartesian_field(CartesianField::Y, value)
                }
                Message::CartesianZChanged(value) => {
                    state.update_cartesian_field(CartesianField::Z, value)
                }
                Message::CloseError => state.close_error(),
                Message::CloseModal => state.close_modal(),
                Message::ConstantGravityXChanged(string) => {
                    state.constant_gravity_x_changed(string)
                }
                Message::ConstantGravityYChanged(string) => {
                    state.constant_gravity_y_changed(string)
                }
                Message::ConstantGravityZChanged(string) => {
                    state.constant_gravity_z_changed(string)
                }
                Message::CursorMoved(cursor) => state.cursor_moved(cursor),
                Message::CuboidLengthInputChanged(value) => {
                    state.update_cuboid_field(CuboidField::Length, value)
                }
                Message::CuboidWidthInputChanged(value) => {
                    state.update_cuboid_field(CuboidField::Width, value)
                }
                Message::CuboidHeightInputChanged(value) => {
                    state.update_cuboid_field(CuboidField::Height, value)
                }
                Message::CylindricalAzimuthChanged(value) => {
                    state.update_cylindrical_field(CylindricalField::Azimuth, value)
                }
                Message::CylindricalHeightChanged(value) => {
                    state.update_cylindrical_field(CylindricalField::Height, value)
                }
                Message::CylindricalRadiusChanged(value) => {
                    state.update_cylindrical_field(CylindricalField::Radius, value)
                }
                Message::DeletePressed => state.delete_pressed(),
                Message::EnterPressed => state.enter_pressed(),
                Message::EulerAngleSequenceChanged(sequence) => {
                    state.update_euler_angle_sequence(sequence)
                }
                Message::EulerAnglePhiChanged(value) => {
                    state.update_euler_angle_field(EulerAnglesField::Phi, value)
                }
                Message::EulerAngleThetaChanged(value) => {
                    state.update_euler_angle_field(EulerAnglesField::Theta, value)
                }
                Message::EulerAnglePsiChanged(value) => {
                    state.update_euler_angle_field(EulerAnglesField::Psi, value)
                }
                Message::FontLoaded(_) => Command::none(),
                Message::GeometrySelected(geometry) => state.geometry_selected(geometry),
                Message::GravityModelSelected(gravity) => state.gravity_model_selected(gravity),
                Message::LeftButtonPressed(cursor) => state.left_button_pressed(cursor),
                Message::LeftButtonReleased(cursor) => state.left_button_released(cursor),

                Message::Loaded(_) => Command::none(),
                Message::MiddleButtonPressed(cursor) => state.middle_button_pressed(cursor),

                Message::PlotSimSelected(string) => state.plot_sim_selected(string),
                Message::PlotStateSelected(string) => state.plot_state_selected(string),
                Message::PlotComponentSelected(string) => state.plot_component_selected(string),

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

                Message::QuaternionWChanged(value) => {
                    state.update_quaternion_field(QuaternionField::W, value)
                }
                Message::QuaternionXChanged(value) => {
                    state.update_quaternion_field(QuaternionField::X, value)
                }
                Message::QuaternionYChanged(value) => {
                    state.update_quaternion_field(QuaternionField::Y, value)
                }
                Message::QuaternionZChanged(value) => {
                    state.update_quaternion_field(QuaternionField::Z, value)
                }
                Message::ResultSelected(_) => Command::none(),
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
                Message::RightButtonPressed(cursor) => state.right_button_pressed(cursor),
                Message::RightButtonReleased(cursor) => state.right_button_released(cursor),
                Message::RotationMatrixE11Changed(value) => {
                    state.update_rotation_matrix_field(RotationMatrixField::E11, value)
                }
                Message::RotationMatrixE12Changed(value) => {
                    state.update_rotation_matrix_field(RotationMatrixField::E12, value)
                }
                Message::RotationMatrixE13Changed(value) => {
                    state.update_rotation_matrix_field(RotationMatrixField::E13, value)
                }
                Message::RotationMatrixE21Changed(value) => {
                    state.update_rotation_matrix_field(RotationMatrixField::E21, value)
                }
                Message::RotationMatrixE22Changed(value) => {
                    state.update_rotation_matrix_field(RotationMatrixField::E22, value)
                }
                Message::RotationMatrixE23Changed(value) => {
                    state.update_rotation_matrix_field(RotationMatrixField::E23, value)
                }
                Message::RotationMatrixE31Changed(value) => {
                    state.update_rotation_matrix_field(RotationMatrixField::E31, value)
                }
                Message::RotationMatrixE32Changed(value) => {
                    state.update_rotation_matrix_field(RotationMatrixField::E32, value)
                }
                Message::RotationMatrixE33Changed(value) => {
                    state.update_rotation_matrix_field(RotationMatrixField::E33, value)
                }

                Message::SaveComponent => state.save_component(),
                Message::SimDtChanged(string) => state.simdiv.dt_changed(string),
                Message::SimStartTimeChanged(string) => state.simdiv.start_time_changed(string),
                Message::SimStopTimeChanged(string) => state.simdiv.stop_time_changed(string),
                Message::SimNameChanged(string) => state.simdiv.name_changed(string),
                Message::Simulate => state.simulate(),
                Message::SphericalAzimuthChanged(value) => {
                    state.update_spherical_field(SphericalField::Azimuth, value)
                }
                Message::SphericalInclinationChanged(value) => {
                    state.update_spherical_field(SphericalField::Inclination, value)
                }
                Message::SphericalRadiusChanged(value) => {
                    state.update_spherical_field(SphericalField::Radius, value)
                }
                Message::TabPressed => state.tab_pressed(),
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
                Message::TransformSelected(transform) => state.transform_selected(transform),
                Message::TransformTranslationSelected(transform) => {
                    state.transform_translation_selected(transform)
                }
                Message::TransformRotationSelected(transform) => {
                    state.transform_rotation_selected(transform)
                }
                Message::TwoBodyCustomMuChanged(string) => state.two_body_custom_mu_changed(string),
                Message::TwoBodyModelSelected(model) => state.two_body_model_changed(model),
                Message::WheelScrolled(delta) => state.wheel_scrolled(delta),

                Message::WindowResized(size) => state.window_resized(size),
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
            DummyComponent::Gravity => Some(create_gravity_modal(
                &state.nodebar.dummies.gravity,
                &state.nodebar.dummies.constant_gravity,
                &state.nodebar.dummies.two_body,
                &state.nodebar.dummies.two_body_custom,
            )),
            DummyComponent::Revolute => {
                Some(create_revolute_modal(&state.nodebar.dummies.revolute))
            }
            DummyComponent::Prismatic => {
                Some(create_prismatic_modal(&state.nodebar.dummies.prismatic))
            }
            DummyComponent::Transform => Some(create_transform_modal(
                &state.nodebar.dummies.transform,
                &state.nodebar.dummies.aligned_axes,
                &state.nodebar.dummies.cartesian,
                &state.nodebar.dummies.cylindrical,
                &state.nodebar.dummies.euler_angles,
                &state.nodebar.dummies.quaternion,
                &state.nodebar.dummies.rotation_matrix,
                &state.nodebar.dummies.spherical,
            )),
        }
    } else {
        None
    };

    modal(underlay, overlay)
        .on_esc(Message::CloseModal)
        .align_y(alignment::Vertical::Center)
        .into()
}
