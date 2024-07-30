use super::dummies::{
    DummyAlignedAxes, DummyBase, DummyBody, DummyCartesian, DummyComponent, DummyConstantGravity,
    DummyCuboid, DummyCylindrical, DummyEulerAngles, DummyGravity, DummyPrismatic, DummyQuaternion,
    DummyRevolute, DummyRotationMatrix, DummySpherical, DummyTransform, DummyTwoBodyCustom,
    DummyTwoBodyGravity, GeometryPickList, RotationPickList, TransformPickList,
    TranslationPickList,
};
use crate::{
    ui::{errors::Errors, theme::Theme},
    Message,
};
use iced::{
    widget::{button, pick_list, text, text_input, Column, Row},
    Element, Length,
};
use iced_aw::widgets::card;

use uuid::Uuid;
#[derive(Debug, Clone, Copy)]
pub struct ActiveModal {
    pub dummy_type: DummyComponent,
    pub component_id: Option<Uuid>, // None if new component, id if editing component
}

impl ActiveModal {
    pub fn new(dummy_type: DummyComponent, component_id: Option<Uuid>) -> Self {
        Self {
            dummy_type,
            component_id,
        }
    }
}

fn create_modal<'a>(
    header: &'a str,
    content: Element<'a, Message, Theme>,
) -> Element<'a, Message, Theme> {
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

    card(header, content).foot(footer).max_width(500.0).into()
}

pub fn create_base_modal(_base: &DummyBase) -> Element<Message, Theme> {
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

pub fn create_body_modal<'a>(
    body: &'a DummyBody,
    cuboid: &'a DummyCuboid,
) -> Element<'a, Message, Theme> {
    let create_text_input = |label: &str, value: &str, on_input: fn(String) -> Message| {
        Row::new()
            .spacing(10)
            .padding(5)
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
        ))
        .push(
            Row::new()
                .spacing(10)
                .padding(5)
                .push(text("Geometry").width(Length::FillPortion(1)))
                .push(
                    pick_list(
                        &GeometryPickList::ALL[..],
                        Some(body.geometry),
                        Message::GeometrySelected,
                    )
                    .width(Length::FillPortion(1)),
                )
                .width(Length::Fill),
        );

    let content = match body.geometry {
        GeometryPickList::None => content, //nothing to do
        GeometryPickList::Cuboid => content
            .push(create_text_input(
                "length",
                &cuboid.length,
                Message::CuboidLengthInputChanged,
            ))
            .push(create_text_input(
                "width",
                &cuboid.width,
                Message::CuboidWidthInputChanged,
            ))
            .push(create_text_input(
                "height",
                &cuboid.height,
                Message::CuboidHeightInputChanged,
            )),
    };

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

pub fn create_gravity_modal<'a>(
    dummy_gravity: &'a DummyGravity,
    dummy_constant: &'a DummyConstantGravity,
    dummy_two_body: &'a DummyTwoBodyGravity,
    dummy_two_body_custom: &'a DummyTwoBodyCustom,
) -> Element<'a, Message, Theme> {
    create_modal(
        "Gravity",
        dummy_gravity.content(dummy_constant, dummy_two_body, dummy_two_body_custom),
    )
}

pub fn create_revolute_modal(joint: &DummyRevolute) -> Element<Message, Theme> {
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
        .push(create_text_input("name", &joint.name, |string| {
            Message::RevoluteNameInputChanged(string)
        }))
        .push(create_text_input("theta", &joint.theta, |string| {
            Message::RevoluteThetaInputChanged(string)
        }))
        .push(create_text_input("omega", &joint.omega, |string| {
            Message::RevoluteOmegaInputChanged(string)
        }))
        .push(create_text_input(
            "constant force",
            &joint.constant_force,
            |string| Message::RevoluteConstantForceInputChanged(string),
        ))
        .push(create_text_input("damping", &joint.damping, |string| {
            Message::RevoluteDampingInputChanged(string)
        }))
        .push(create_text_input(
            "spring constant",
            &joint.spring_constant,
            |string| Message::RevoluteSpringConstantInputChanged(string),
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

pub fn create_prismatic_modal(joint: &DummyPrismatic) -> Element<Message, Theme> {
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
        .push(create_text_input("name", &joint.name, |string| {
            Message::PrismaticNameInputChanged(string)
        }))
        .push(create_text_input("position", &joint.position, |string| {
            Message::PrismaticPositionInputChanged(string)
        }))
        .push(create_text_input("velocity", &joint.velocity, |string| {
            Message::PrismaticVelocityInputChanged(string)
        }))
        .push(create_text_input(
            "constant force",
            &joint.constant_force,
            |string| Message::PrismaticConstantForceInputChanged(string),
        ))
        .push(create_text_input("damping", &joint.damping, |string| {
            Message::PrismaticDampingInputChanged(string)
        }))
        .push(create_text_input(
            "spring constant",
            &joint.spring_constant,
            |string| Message::PrismaticSpringConstantInputChanged(string),
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

    card("Prismatic Information", content)
        .foot(footer)
        .max_width(500.0)
        .into()
}

pub fn create_error_modal(error: Errors) -> Element<'static, Message, Theme> {
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
        .style(crate::ui::theme::Card::Error)
        .into()
}

pub fn create_transform_modal<'a>(
    transform: &'a DummyTransform,
    aligned_axes: &'a DummyAlignedAxes,
    cartesian: &'a DummyCartesian,
    cylindrical: &'a DummyCylindrical,
    euler_angles: &'a DummyEulerAngles,
    quaternion: &'a DummyQuaternion,
    rotation_matrix: &'a DummyRotationMatrix,
    spherical: &'a DummySpherical,
) -> Element<'a, Message, Theme> {
    let content = Column::new().push(
        Row::new()
            .spacing(10)
            .padding(5)
            .push(text("Type").width(Length::FillPortion(1)))
            .push(
                pick_list(
                    &TransformPickList::ALL[..],
                    Some(transform.transform_type),
                    Message::TransformSelected,
                )
                .width(Length::FillPortion(1)),
            )
            .width(Length::Fill),
    );

    let content = match transform.transform_type {
        TransformPickList::Identity => content, // no further content needed
        TransformPickList::Custom => {
            // add rotation pick list
            let content = content.push(
                Row::new()
                    .spacing(10)
                    .padding(5)
                    .push(text("Rotation").width(Length::FillPortion(1)))
                    .push(
                        pick_list(
                            &RotationPickList::ALL[..],
                            Some(transform.rotation),
                            Message::TransformRotationSelected,
                        )
                        .width(Length::FillPortion(1)),
                    )
                    .width(Length::Fill),
            );

            // add content based on the rotation
            let content = match transform.rotation {
                RotationPickList::Identity => content, //no further content needed
                RotationPickList::Quaternion => content.push(quaternion.content()),
                RotationPickList::RotationMatrix => content.push(rotation_matrix.content()),
                RotationPickList::AlignedAxes => content.push(aligned_axes.content()),
                RotationPickList::EulerAngles => content.push(euler_angles.content()),
            };

            // add translation pick list
            let content = content.push(
                Row::new()
                    .spacing(10)
                    .padding(5)
                    .push(text("Translation").width(Length::FillPortion(1)))
                    .push(
                        pick_list(
                            &TranslationPickList::ALL[..],
                            Some(transform.translation),
                            Message::TransformTranslationSelected,
                        )
                        .width(Length::FillPortion(1)),
                    )
                    .width(Length::Fill),
            );

            // add content based on the rotation
            let content = match transform.translation {
                TranslationPickList::Zero => content, //no further content needed
                TranslationPickList::Cartesian => content.push(cartesian.content()),
                TranslationPickList::Cylindrical => content.push(cylindrical.content()),
                TranslationPickList::Spherical => content.push(spherical.content()),
            };

            content.into()
        }
    };

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

    card("Transform Information", content)
        .foot(footer)
        .max_width(500.0)
        .into()
}

pub fn create_text_input<'a>(
    label: &'a str,
    value: &'a str,
    on_input: fn(String) -> Message,
) -> Element<'a, Message, Theme> {
    Row::new()
        .spacing(10)
        .padding(5)
        .push(text(label).width(Length::FillPortion(1)))
        .push(
            text_input(label, value)
                .on_input(on_input)
                .on_submit(Message::SaveComponent)
                .width(Length::FillPortion(4)),
        )
        .width(Length::Fill)
        .into()
}
