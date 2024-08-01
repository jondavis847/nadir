use aerospace::gravity::{self, ConstantGravity, Gravity, TwoBodyGravity};
use coordinate_systems::{
    cartesian::Cartesian,
    cylindrical::{self, Cylindrical},
    spherical::Spherical,
    CoordinateSystem,
};
use geometry::cuboid::Cuboid;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    aerospace::MultibodyGravity, base::Base, body::Body, joint::{
        prismatic::{Prismatic, PrismaticState},
        revolute::{Revolute, RevoluteState},
        Joint, JointParameters,
    }, MultibodyTrait
};
use nalgebra::Matrix3;
use rotations::{
    axes::{AlignedAxes, Axis, AxisPair},
    euler_angles::{EulerAngles, EulerSequence},
    quaternion::Quaternion,
    rotation_matrix::RotationMatrix,
    Rotation,
};
use transforms::Transform;

use crate::{
    ui::{modals::create_text_input, theme::Theme},
    Message,
};
use iced::{
    widget::{pick_list, text, text_input, Column, Row},
    Element, Length,
};

#[derive(Debug, Default, Clone)]
pub struct Dummies {
    pub base: DummyBase,
    pub body: DummyBody,
    pub constant_gravity: DummyConstantGravity,
    pub gravity: DummyGravity,
    pub transform: DummyTransform,
    pub prismatic: DummyPrismatic,
    pub revolute: DummyRevolute,
    pub cartesian: DummyCartesian,
    pub cylindrical: DummyCylindrical,
    pub spherical: DummySpherical,
    pub cuboid: DummyCuboid,
    pub quaternion: DummyQuaternion,
    pub rotation_matrix: DummyRotationMatrix,
    pub two_body: DummyTwoBodyGravity,
    pub two_body_custom: DummyTwoBodyCustom,
    pub euler_angles: DummyEulerAngles,
    pub aligned_axes: DummyAlignedAxes,
}

/// DummyComponents are like MultibodyComponents but with String fields
/// for editing in the text inputs rather than numeric values
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DummyComponent {
    Base,
    Body,
    Gravity,
    Revolute,
    Prismatic,
    Transform,
}

#[derive(Debug, Default, Clone)]
pub struct DummyBase {
    pub name: String,
}

impl DummyBase {
    pub fn clear(&mut self) {
        self.name = "".to_string();
    }

    pub fn set_values_for(&self, base: &mut Base) {
        base.set_name(self.name.clone());
    }

    pub fn get_values_from(&mut self, base: &Base) {
        self.name = base.get_name().to_string();
    }

    pub fn to_base(&self) -> Base {
        Base::new(self.name.as_str())
    }
}

#[derive(Default, Debug, Clone)]
pub struct DummyBody {
    pub name: String,
    pub mass: String,
    pub cmx: String,
    pub cmy: String,
    pub cmz: String,
    pub ixx: String,
    pub iyy: String,
    pub izz: String,
    pub ixy: String,
    pub ixz: String,
    pub iyz: String,
    pub geometry: GeometryPickList,
}

impl DummyBody {
    pub fn clear(&mut self) {
        self.name = String::new();
        self.mass = String::new();
        self.cmx = String::new();
        self.cmy = String::new();
        self.cmz = String::new();
        self.ixx = String::new();
        self.iyy = String::new();
        self.izz = String::new();
        self.ixy = String::new();
        self.ixz = String::new();
        self.iyz = String::new();
        self.geometry = GeometryPickList::None;
    }

    pub fn get_values_from(&mut self, body: &Body) {
        let mp = body.mass_properties;

        self.name = body.name.clone();
        self.mass = mp.mass.to_string();
        self.cmx = mp.center_of_mass.get_cmx().to_string();
        self.cmy = mp.center_of_mass.get_cmy().to_string();
        self.cmz = mp.center_of_mass.get_cmz().to_string();
        self.ixx = mp.inertia.ixx.to_string();
        self.iyy = mp.inertia.iyy.to_string();
        self.izz = mp.inertia.izz.to_string();
        self.ixy = mp.inertia.ixy.to_string();
        self.ixz = mp.inertia.ixz.to_string();
        self.iyz = mp.inertia.iyz.to_string();
    }

    pub fn set_values_for(&self, body: &mut Body) {
        body.set_name(self.name.clone());
        let cmx = self.cmx.parse::<f64>().unwrap();
        let cmy = self.cmy.parse::<f64>().unwrap();
        let cmz = self.cmz.parse::<f64>().unwrap();
        let ixx = self.ixx.parse::<f64>().unwrap();
        let iyy = self.iyy.parse::<f64>().unwrap();
        let izz = self.izz.parse::<f64>().unwrap();
        let ixy = self.ixy.parse::<f64>().unwrap();
        let ixz = self.ixz.parse::<f64>().unwrap();
        let iyz = self.iyz.parse::<f64>().unwrap();

        let mass = self.mass.parse::<f64>().unwrap();
        let cm = CenterOfMass::new(cmx, cmy, cmz);
        let inertia = Inertia::new(ixx, iyy, izz, ixy, ixz, iyz).unwrap();
        let mp = MassProperties::new(mass, cm, inertia).unwrap();

        body.mass_properties = mp;
    }

    pub fn to_body(&self) -> Body {
        let cmx = self.cmx.parse::<f64>().unwrap_or(0.0);
        let cmy = self.cmy.parse::<f64>().unwrap_or(0.0);
        let cmz = self.cmz.parse::<f64>().unwrap_or(0.0);
        let ixx = self.ixx.parse::<f64>().unwrap_or(1.0);
        let iyy = self.iyy.parse::<f64>().unwrap_or(1.0);
        let izz = self.izz.parse::<f64>().unwrap_or(1.0);
        let ixy = self.ixy.parse::<f64>().unwrap_or(0.0);
        let ixz = self.ixz.parse::<f64>().unwrap_or(0.0);
        let iyz = self.iyz.parse::<f64>().unwrap_or(0.0);

        let mass = self.mass.parse::<f64>().unwrap_or(1.0);
        let cm = CenterOfMass::new(cmx, cmy, cmz);
        let inertia = Inertia::new(ixx, iyy, izz, ixy, ixz, iyz).unwrap();
        let mp = MassProperties::new(mass, cm, inertia).unwrap();
        Body::new(self.name.as_str(), mp).unwrap()
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub enum TransformPickList {
    #[default]
    Identity,
    Custom,
}

impl TransformPickList {
    pub const ALL: [TransformPickList; 2] =
        [TransformPickList::Identity, TransformPickList::Custom];

    pub fn content(&self) -> Element<Message, crate::ui::theme::Theme> {
        let content = Row::new()
            .spacing(10)
            .padding(5)
            .push(text("Transform Type").width(Length::FillPortion(1)))
            .push(
                pick_list(
                    &TransformPickList::ALL[..],
                    Some(*self),
                    Message::TransformSelected,
                )
                .width(Length::FillPortion(1)),
            )
            .width(Length::Fill);

        let content = match self {
            TransformPickList::Identity => content,
            TransformPickList::Custom => content,
        };
        content.into()
    }
}

impl std::fmt::Display for TransformPickList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                TransformPickList::Identity => "Identity",
                TransformPickList::Custom => "Custom",
            }
        )
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct DummyTransform {
    pub transform_type: TransformPickList,
    pub rotation: RotationPickList,
    pub translation: TranslationPickList,
}
impl DummyTransform {
    pub fn clear(&mut self) {
        self.transform_type = TransformPickList::Identity;
        self.rotation = RotationPickList::Identity;
        self.translation = TranslationPickList::Zero;
    }

    pub fn get_values_from(
        &mut self,
        transform: &Transform,
        dummy_cartesian: &mut DummyCartesian,
        dummy_cylindrical: &mut DummyCylindrical,
        dummy_euler_angles: &mut DummyEulerAngles,
        dummy_quaternion: &mut DummyQuaternion,
        dummy_rotation_matrix: &mut DummyRotationMatrix,
        dummy_spherical: &mut DummySpherical,
    ) {
        match transform.rotation {
            Rotation::EulerAngles(euler_angles) => {
                self.rotation = RotationPickList::EulerAngles;
                dummy_euler_angles.get_values_from(&euler_angles);
            }
            Rotation::Quaternion(quaternion) => {
                self.rotation = RotationPickList::Quaternion;
                dummy_quaternion.get_values_from(&quaternion);
            }
            Rotation::RotationMatrix(rotation_matrix) => {
                self.rotation = RotationPickList::RotationMatrix;
                dummy_rotation_matrix.get_values_from(&rotation_matrix);
            }
        };

        match transform.translation {
            CoordinateSystem::Cartesian(cartesian) => {
                self.translation = TranslationPickList::Cartesian;
                dummy_cartesian.get_values_from(&cartesian);
            }
            CoordinateSystem::Cylindrical(cylindrical) => {
                self.translation = TranslationPickList::Cylindrical;
                dummy_cylindrical.get_values_from(&cylindrical);
            }
            CoordinateSystem::Spherical(spherical) => {
                self.translation = TranslationPickList::Spherical;
                dummy_spherical.get_values_from(&spherical);
            }
        }
    }

    pub fn set_values_for(&self, transform: &mut Transform, dummies: &Dummies) {
        match self.transform_type {
            TransformPickList::Identity => *transform = Transform::IDENTITY,
            TransformPickList::Custom => {
                let rotation = match self.rotation {
                    RotationPickList::Identity => Rotation::IDENTITY,
                    RotationPickList::Quaternion => {
                        Rotation::from(dummies.quaternion.to_quaternion())
                    }
                    RotationPickList::RotationMatrix => {
                        Rotation::from(dummies.rotation_matrix.to_rotation_matrix())
                    }
                    RotationPickList::AlignedAxes => {
                        Rotation::from(dummies.aligned_axes.to_aligned_axes())
                    }
                    RotationPickList::EulerAngles => {
                        Rotation::from(dummies.euler_angles.to_euler_angles())
                    }
                };

                let translation = match self.translation {
                    TranslationPickList::Zero => CoordinateSystem::ZERO,
                    TranslationPickList::Cartesian => {
                        CoordinateSystem::from(dummies.cartesian.to_cartesian())
                    }
                    TranslationPickList::Cylindrical => {
                        CoordinateSystem::from(dummies.cylindrical.to_cylindrical())
                    }
                    TranslationPickList::Spherical => {
                        CoordinateSystem::from(dummies.spherical.to_spherical())
                    }
                };

                *transform = Transform::new(rotation, translation);
            }
        }
    }

    pub fn to_transform(&self, dummies: &Dummies) -> Transform {
        match self.transform_type {
            TransformPickList::Identity => Transform::IDENTITY,
            TransformPickList::Custom => {
                let rotation: Rotation = match self.rotation {
                    RotationPickList::Identity => Rotation::IDENTITY,
                    RotationPickList::AlignedAxes => dummies.aligned_axes.to_aligned_axes().into(),
                    RotationPickList::EulerAngles => dummies.euler_angles.to_euler_angles().into(),
                    RotationPickList::Quaternion => dummies.quaternion.to_quaternion().into(),
                    RotationPickList::RotationMatrix => {
                        dummies.rotation_matrix.to_rotation_matrix().into()
                    }
                };

                let translation: CoordinateSystem = match self.translation {
                    TranslationPickList::Zero => CoordinateSystem::ZERO,
                    TranslationPickList::Cartesian => dummies.cartesian.to_cartesian().into(),
                    TranslationPickList::Cylindrical => dummies.cylindrical.to_cylindrical().into(),
                    TranslationPickList::Spherical => dummies.spherical.to_spherical().into(),
                };

                Transform::new(rotation, translation)
            }
        }
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub enum RotationPickList {
    #[default]
    Identity,
    AlignedAxes,
    EulerAngles,
    Quaternion,
    RotationMatrix,
}

impl RotationPickList {
    pub const ALL: [RotationPickList; 5] = [
        RotationPickList::Identity,
        RotationPickList::AlignedAxes,
        RotationPickList::EulerAngles,
        RotationPickList::Quaternion,
        RotationPickList::RotationMatrix,
    ];
}

impl std::fmt::Display for RotationPickList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                RotationPickList::Identity => "Identity",
                RotationPickList::AlignedAxes => "Aligned Axes",
                RotationPickList::EulerAngles => "Euler Angles",
                RotationPickList::Quaternion => "Quaternion",
                RotationPickList::RotationMatrix => "Rotation Matrix",
            }
        )
    }
}

#[derive(Default, Debug, Clone, Copy, PartialEq)]
pub enum TranslationPickList {
    #[default]
    Zero,
    Cartesian,
    Cylindrical,
    Spherical,
}

impl TranslationPickList {
    pub const ALL: [TranslationPickList; 4] = [
        TranslationPickList::Zero,
        TranslationPickList::Cartesian,
        TranslationPickList::Cylindrical,
        TranslationPickList::Spherical,
    ];
}

impl std::fmt::Display for TranslationPickList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                TranslationPickList::Zero => "Zero",
                TranslationPickList::Cartesian => "Cartesian",
                TranslationPickList::Cylindrical => "Cylindrical",
                TranslationPickList::Spherical => "Spherical",
            }
        )
    }
}

#[derive(Default, Debug, Clone)]
pub struct DummyRevolute {
    pub constant_force: String,
    pub damping: String,
    pub name: String,
    pub omega: String,
    pub spring_constant: String,
    pub theta: String,
}

impl DummyRevolute {
    pub fn clear(&mut self) {
        self.name = String::new();
        self.theta = String::new();
        self.omega = String::new();
        self.spring_constant = String::new();
        self.damping = String::new();
        self.constant_force = String::new();
    }
    pub fn get_values_from(&mut self, rev: &Revolute) {
        self.name = rev.get_name().to_string();
        self.theta = rev.state.theta.to_string();
        self.omega = rev.state.omega.to_string();
        self.spring_constant = rev.parameters.spring_constant.to_string();
        self.damping = rev.parameters.damping.to_string();
        self.constant_force = rev.parameters.constant_force.to_string();
    }

    pub fn set_values_for(&self, rev: &mut Revolute) {
        rev.set_name(self.name.clone());
        rev.state.theta = self.theta.parse::<f64>().unwrap_or(0.0);
        rev.state.omega = self.omega.parse::<f64>().unwrap_or(0.0);
        rev.parameters.spring_constant = self.spring_constant.parse::<f64>().unwrap_or(0.0);
        rev.parameters.damping = self.damping.parse::<f64>().unwrap_or(0.0);
        rev.parameters.constant_force = self.constant_force.parse::<f64>().unwrap_or(0.0);
    }

    pub fn to_joint(&self) -> Joint {
        let state = RevoluteState::new(
            self.theta.parse::<f64>().unwrap_or(0.0),
            self.omega.parse::<f64>().unwrap_or(0.0),
        );
        let parameters = JointParameters::new(
            self.constant_force.parse::<f64>().unwrap_or(0.0),
            self.damping.parse::<f64>().unwrap_or(0.0),
            self.spring_constant.parse::<f64>().unwrap_or(0.0),
        );
        let revolute = Revolute::new(self.name.as_str(), parameters, state);
        Joint::Revolute(revolute)
    }
}

#[derive(Default, Debug, Clone)]
pub struct DummyPrismatic {
    pub constant_force: String,
    pub damping: String,
    pub name: String,
    pub position: String,
    pub spring_constant: String,
    pub velocity: String,
}

impl DummyPrismatic {
    pub fn clear(&mut self) {
        self.name = String::new();
        self.position = String::new();
        self.velocity = String::new();
        self.spring_constant = String::new();
        self.damping = String::new();
        self.constant_force = String::new();
    }
    pub fn get_values_from(&mut self, rev: &Prismatic) {
        self.name = rev.get_name().to_string();
        self.position = rev.state.position.to_string();
        self.velocity = rev.state.velocity.to_string();
        self.spring_constant = rev.parameters.spring_constant.to_string();
        self.damping = rev.parameters.damping.to_string();
        self.constant_force = rev.parameters.constant_force.to_string();
    }

    pub fn set_values_for(&self, rev: &mut Prismatic) {
        rev.set_name(self.name.clone());
        rev.state.position = self.position.parse::<f64>().unwrap_or(0.0);
        rev.state.velocity = self.velocity.parse::<f64>().unwrap_or(0.0);
        rev.parameters.spring_constant = self.spring_constant.parse::<f64>().unwrap_or(0.0);
        rev.parameters.damping = self.damping.parse::<f64>().unwrap_or(0.0);
        rev.parameters.constant_force = self.constant_force.parse::<f64>().unwrap_or(0.0);
    }

    pub fn to_joint(&self) -> Joint {
        let state = PrismaticState::new(
            self.position.parse::<f64>().unwrap_or(0.0),
            self.velocity.parse::<f64>().unwrap_or(0.0),
        );
        let parameters = JointParameters::new(
            self.constant_force.parse::<f64>().unwrap_or(0.0),
            self.damping.parse::<f64>().unwrap_or(0.0),
            self.spring_constant.parse::<f64>().unwrap_or(0.0),
        );
        let prismatic = Prismatic::new(self.name.as_str(), parameters, state);
        Joint::Prismatic(prismatic)
    }
}

#[derive(Debug, Clone)]
pub enum DummyGeometry {
    Cuboid(DummyCuboid),
}

#[derive(Default, Debug, Clone)]
pub struct DummyCuboid {
    pub length: String,
    pub width: String,
    pub height: String,
}

impl DummyCuboid {
    pub fn new() -> Self {
        Self {
            length: String::new(),
            width: String::new(),
            height: String::new(),
        }
    }
    pub fn clear(&mut self) {
        self.length = String::new();
        self.width = String::new();
        self.height = String::new();
    }

    pub fn get_values_from(&mut self, cuboid: &Cuboid) {
        self.length = cuboid.length.to_string();
        self.width = cuboid.width.to_string();
        self.height = cuboid.height.to_string();
    }

    pub fn set_values_for(&self, cuboid: &mut Cuboid) {
        cuboid.length = self.length.parse::<f32>().unwrap();
        cuboid.width = self.width.parse::<f32>().unwrap();
        cuboid.height = self.height.parse::<f32>().unwrap();
    }

    pub fn to_cuboid(&self) -> Cuboid {
        let length = self.length.parse::<f32>().unwrap_or(1.0);
        let width = self.width.parse::<f32>().unwrap_or(1.0);
        let height = self.height.parse::<f32>().unwrap_or(1.0);

        Cuboid {
            length,
            width,
            height,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum GeometryPickList {
    #[default]
    None,
    Cuboid,
}

impl GeometryPickList {
    pub const ALL: [GeometryPickList; 2] = [GeometryPickList::None, GeometryPickList::Cuboid];
}

impl std::fmt::Display for GeometryPickList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                GeometryPickList::None => "None",
                GeometryPickList::Cuboid => "Cuboid",
            }
        )
    }
}

#[derive(Debug, Clone)]
pub enum DummyRotations {
    Quaternion(DummyQuaternion),
    RotationMatrix(DummyRotationMatrix),
    EulerAngles(DummyEulerAngles),
    AlignedAxes(DummyAlignedAxes),
}

#[derive(Default, Debug, Clone)]
pub struct DummyQuaternion {
    pub w: String,
    pub x: String,
    pub y: String,
    pub z: String,
}

impl DummyQuaternion {
    pub fn new() -> Self {
        Self {
            w: String::new(),
            x: String::new(),
            y: String::new(),
            z: String::new(),
        }
    }
    pub fn clear(&mut self) {
        self.w = String::new();
        self.x = String::new();
        self.y = String::new();
        self.z = String::new();
    }

    pub fn content<'a>(&'a self) -> Element<'a, Message, Theme> {
        Column::new()
            .push(create_text_input(
                "w",
                self.w.as_str(),
                Message::QuaternionWChanged,
            ))
            .push(create_text_input(
                "x",
                self.x.as_str(),
                Message::QuaternionXChanged,
            ))
            .push(create_text_input(
                "y",
                self.y.as_str(),
                Message::QuaternionYChanged,
            ))
            .push(create_text_input(
                "z",
                self.z.as_str(),
                Message::QuaternionZChanged,
            ))
            .into()
    }

    pub fn get_values_from(&mut self, quaternion: &Quaternion) {
        self.w = quaternion.s.to_string();
        self.x = quaternion.x.to_string();
        self.y = quaternion.y.to_string();
        self.z = quaternion.z.to_string();
    }

    pub fn set_values_for(&self, quaternion: &mut Quaternion) {
        quaternion.s = self.w.parse::<f64>().unwrap();
        quaternion.x = self.x.parse::<f64>().unwrap();
        quaternion.y = self.y.parse::<f64>().unwrap();
        quaternion.z = self.z.parse::<f64>().unwrap();
    }

    pub fn to_quaternion(&self) -> Quaternion {
        // ensure that either all entries are empty or all entries are full
        let all_empty =
            self.w.is_empty() && self.x.is_empty() && self.y.is_empty() && self.z.is_empty();
        let all_full =
            !self.w.is_empty() && !self.x.is_empty() && !self.y.is_empty() && !self.z.is_empty();

        if !all_empty && !all_full {
            panic!("quaternion values must either be all empty or all provided");
        }

        let w = self.w.parse::<f64>().unwrap_or(1.0);
        let x = self.x.parse::<f64>().unwrap_or(0.0);
        let y = self.y.parse::<f64>().unwrap_or(0.0);
        let z = self.z.parse::<f64>().unwrap_or(0.0);

        Quaternion::new(x, y, z, w)
    }
}

#[derive(Default, Debug, Clone)]
pub struct DummyRotationMatrix {
    pub e11: String,
    pub e12: String,
    pub e13: String,
    pub e21: String,
    pub e22: String,
    pub e23: String,
    pub e31: String,
    pub e32: String,
    pub e33: String,
}

impl DummyRotationMatrix {
    pub fn clear(&mut self) {
        self.e11 = String::new();
        self.e12 = String::new();
        self.e13 = String::new();
        self.e21 = String::new();
        self.e22 = String::new();
        self.e23 = String::new();
        self.e31 = String::new();
        self.e32 = String::new();
        self.e33 = String::new();
    }

    pub fn content<'a>(&'a self) -> Element<'a, Message, Theme> {
        Column::new()
            .push(
                Row::new()
                    .push(
                        text_input("1,1", &self.e11)
                            .on_input(Message::RotationMatrixE11Changed)
                            .width(Length::FillPortion(3)),
                    )
                    .push(
                        text_input("1,2", &self.e12)
                            .on_input(Message::RotationMatrixE12Changed)
                            .width(Length::FillPortion(3)),
                    )
                    .push(
                        text_input("1,3", &self.e13)
                            .on_input(Message::RotationMatrixE13Changed)
                            .width(Length::FillPortion(3)),
                    ),
            )
            .push(
                Row::new()
                    .push(
                        text_input("2,1", &self.e21)
                            .on_input(Message::RotationMatrixE21Changed)
                            .width(Length::FillPortion(3)),
                    )
                    .push(
                        text_input("2,2", &self.e22)
                            .on_input(Message::RotationMatrixE22Changed)
                            .width(Length::FillPortion(3)),
                    )
                    .push(
                        text_input("2,3", &self.e23)
                            .on_input(Message::RotationMatrixE23Changed)
                            .width(Length::FillPortion(3)),
                    ),
            )
            .push(
                Row::new()
                    .push(
                        text_input("3,1", &self.e31)
                            .on_input(Message::RotationMatrixE31Changed)
                            .width(Length::FillPortion(3)),
                    )
                    .push(
                        text_input("3,2", &self.e32)
                            .on_input(Message::RotationMatrixE32Changed)
                            .width(Length::FillPortion(3)),
                    )
                    .push(
                        text_input("3,3", &self.e33)
                            .on_input(Message::RotationMatrixE33Changed)
                            .width(Length::FillPortion(3)),
                    ),
            )
            .into()
    }

    pub fn get_values_from(&mut self, rotation: &RotationMatrix) {
        self.e11 = rotation.0[(0, 0)].to_string();
        self.e12 = rotation.0[(0, 1)].to_string();
        self.e13 = rotation.0[(0, 2)].to_string();
        self.e21 = rotation.0[(1, 0)].to_string();
        self.e22 = rotation.0[(1, 1)].to_string();
        self.e23 = rotation.0[(1, 2)].to_string();
        self.e31 = rotation.0[(2, 0)].to_string();
        self.e32 = rotation.0[(2, 1)].to_string();
        self.e33 = rotation.0[(2, 2)].to_string();
    }

    pub fn set_values_for(&self, rotation: &mut RotationMatrix) {
        rotation.0[(0, 0)] = self.e11.parse::<f64>().unwrap_or(1.0);
        rotation.0[(0, 1)] = self.e12.parse::<f64>().unwrap_or(0.0);
        rotation.0[(0, 2)] = self.e13.parse::<f64>().unwrap_or(0.0);
        rotation.0[(1, 0)] = self.e21.parse::<f64>().unwrap_or(0.0);
        rotation.0[(1, 1)] = self.e22.parse::<f64>().unwrap_or(1.0);
        rotation.0[(1, 2)] = self.e23.parse::<f64>().unwrap_or(0.0);
        rotation.0[(2, 0)] = self.e31.parse::<f64>().unwrap_or(0.0);
        rotation.0[(2, 1)] = self.e32.parse::<f64>().unwrap_or(0.0);
        rotation.0[(2, 2)] = self.e33.parse::<f64>().unwrap_or(1.0);
    }

    pub fn to_rotation_matrix(&self) -> RotationMatrix {
        let mut rotation = RotationMatrix(Matrix3::identity());
        self.set_values_for(&mut rotation);
        rotation
    }
}

#[derive(Default, Debug, Clone)]
pub struct DummyEulerAngles {
    pub phi: String,
    pub theta: String,
    pub psi: String,
    pub sequence: EulerSequence,
}

impl DummyEulerAngles {
    pub fn clear(&mut self) {
        self.phi = String::new();
        self.theta = String::new();
        self.psi = String::new();
        self.sequence = EulerSequence::ZYX;
    }

    pub fn content<'a>(&'a self) -> Element<'a, Message, Theme> {
        Column::new()
            .push(create_text_input(
                "phi",
                self.phi.as_str(),
                Message::EulerAnglePhiChanged,
            ))
            .push(create_text_input(
                "theta",
                self.theta.as_str(),
                Message::EulerAngleThetaChanged,
            ))
            .push(create_text_input(
                "psi",
                self.psi.as_str(),
                Message::EulerAnglePsiChanged,
            ))
            .push(
                pick_list(
                    &EulerSequence::ALL[..],
                    Some(self.sequence),
                    Message::EulerAngleSequenceChanged,
                )
                .width(Length::FillPortion(1)),
            )
            .into()
    }

    pub fn get_values_from(&mut self, euler_angles: &EulerAngles) {
        self.phi = euler_angles.phi.to_string();
        self.theta = euler_angles.theta.to_string();
        self.psi = euler_angles.psi.to_string();
        self.sequence = euler_angles.sequence.into();
    }

    pub fn set_values_for(&self, euler_angles: &mut EulerAngles) {
        euler_angles.phi = self.phi.parse::<f64>().unwrap();
        euler_angles.theta = self.theta.parse::<f64>().unwrap();
        euler_angles.psi = self.psi.parse::<f64>().unwrap();
        euler_angles.sequence = self.sequence.into();
    }

    pub fn to_euler_angles(&self) -> EulerAngles {
        let phi = self.phi.parse::<f64>().unwrap_or(0.0);
        let theta = self.theta.parse::<f64>().unwrap_or(0.0);
        let psi = self.psi.parse::<f64>().unwrap_or(0.0);
        let sequence = self.sequence;

        EulerAngles::new(phi, theta, psi, sequence)
    }
}

#[derive(Debug, Clone)]
pub struct DummyAlignedAxes {
    pub primary_from: Axis,
    pub primary_to: Axis,
    pub secondary_from: Axis,
    pub secondary_to: Axis,
}

impl Default for DummyAlignedAxes {
    fn default() -> Self {
        Self {
            primary_from: Axis::Xp,
            primary_to: Axis::Xp,
            secondary_from: Axis::Yp,
            secondary_to: Axis::Yp,
        }
    }
}

impl DummyAlignedAxes {
    pub fn clear(&mut self) {
        self.primary_from = Axis::Xp;
        self.primary_to = Axis::Xp;
        self.secondary_from = Axis::Yp;
        self.secondary_to = Axis::Yp;
    }

    pub fn content<'a>(&'a self) -> Element<'a, Message, Theme> {
        Column::new()
            .push(
                Row::new()
                    .push(text("from").width(Length::FillPortion(1)))
                    .push(
                        pick_list(
                            &Axis::ALL[..],
                            Some(self.primary_from),
                            Message::AxisPrimaryFromSelected,
                        )
                        .width(Length::FillPortion(1)),
                    )
                    .push(text("to").width(Length::FillPortion(1)))
                    .push(
                        pick_list(
                            &Axis::ALL[..],
                            Some(self.primary_to),
                            Message::AxisPrimaryToSelected,
                        )
                        .width(Length::FillPortion(1)),
                    ),
            )
            .push(
                Row::new()
                    .push(text("from").width(Length::FillPortion(1)))
                    .push(
                        pick_list(
                            &Axis::ALL[..],
                            Some(self.secondary_from),
                            Message::AxisSecondaryFromSelected,
                        )
                        .width(Length::FillPortion(1)),
                    )
                    .push(text("to").width(Length::FillPortion(1)))
                    .push(
                        pick_list(
                            &Axis::ALL[..],
                            Some(self.secondary_to),
                            Message::AxisSecondaryToSelected,
                        )
                        .width(Length::FillPortion(1)),
                    ),
            )
            .into()
    }

    pub fn get_values_from(&mut self, axes: &AlignedAxes) {
        self.primary_from = axes.primary.old;
        self.primary_to = axes.primary.new;
        self.secondary_from = axes.secondary.old;
        self.secondary_to = axes.secondary.new;
    }

    pub fn set_values_for(&self, axes: &mut AlignedAxes) {
        axes.primary.old = self.primary_from;
        axes.primary.new = self.primary_to;
        axes.secondary.old = self.secondary_from;
        axes.secondary.new = self.secondary_to;
    }

    pub fn to_aligned_axes(&self) -> AlignedAxes {
        AlignedAxes::new(
            AxisPair::new(self.primary_from, self.primary_to),
            AxisPair::new(self.secondary_from, self.secondary_to),
        )
    }
}
#[derive(Debug, Clone, Default)]
pub struct DummyCartesian {
    pub x: String,
    pub y: String,
    pub z: String,
}

impl DummyCartesian {
    pub fn clear(&mut self) {
        self.x = String::new();
        self.y = String::new();
        self.z = String::new();
    }

    pub fn content<'a>(&'a self) -> Element<'a, Message, Theme> {
        Column::new()
            .push(create_text_input(
                "x",
                self.x.as_str(),
                Message::CartesianXChanged,
            ))
            .push(create_text_input(
                "y",
                self.y.as_str(),
                Message::CartesianYChanged,
            ))
            .push(create_text_input(
                "z",
                self.z.as_str(),
                Message::CartesianZChanged,
            ))
            .into()
    }

    pub fn get_values_from(&mut self, cart: &Cartesian) {
        self.x = cart.x.to_string();
        self.y = cart.y.to_string();
        self.z = cart.z.to_string();
    }

    pub fn set_values_for(&self, cart: &mut Cartesian) {
        cart.x = self.x.parse::<f64>().unwrap_or(0.0);
        cart.y = self.y.parse::<f64>().unwrap_or(0.0);
        cart.z = self.z.parse::<f64>().unwrap_or(0.0);
    }

    pub fn to_cartesian(&self) -> Cartesian {
        Cartesian::new(
            self.x.parse::<f64>().unwrap_or(0.0),
            self.y.parse::<f64>().unwrap_or(0.0),
            self.z.parse::<f64>().unwrap_or(0.0),
        )
    }
}

#[derive(Debug, Clone, Default)]
pub struct DummyCylindrical {
    pub radius: String,
    pub azimuth: String,
    pub height: String,
}

impl DummyCylindrical {
    pub fn clear(&mut self) {
        self.radius = String::new();
        self.azimuth = String::new();
        self.height = String::new();
    }

    pub fn content<'a>(&'a self) -> Element<'a, Message, Theme> {
        Column::new()
            .push(create_text_input(
                "radius",
                self.radius.as_str(),
                Message::CylindricalRadiusChanged,
            ))
            .push(create_text_input(
                "azimuth",
                self.azimuth.as_str(),
                Message::CylindricalAzimuthChanged,
            ))
            .push(create_text_input(
                "height",
                self.height.as_str(),
                Message::CylindricalHeightChanged,
            ))
            .into()
    }

    pub fn get_values_from(&mut self, cyl: &Cylindrical) {
        self.radius = cyl.radius.to_string();
        self.azimuth = cyl.azimuth.to_string();
        self.height = cyl.height.to_string();
    }

    pub fn set_values_for(&self, cyl: &mut Cylindrical) {
        cyl.radius = self.radius.parse::<f64>().unwrap_or(0.0);
        cyl.azimuth = self.azimuth.parse::<f64>().unwrap_or(0.0);
        cyl.height = self.height.parse::<f64>().unwrap_or(0.0);
    }

    pub fn to_cylindrical(&self) -> Cylindrical {
        Cylindrical::new(
            self.radius.parse::<f64>().unwrap_or(0.0),
            self.azimuth.parse::<f64>().unwrap_or(0.0),
            self.height.parse::<f64>().unwrap_or(0.0),
        )
    }
}

#[derive(Debug, Clone, Default)]
pub struct DummySpherical {
    pub radius: String,
    pub azimuth: String,
    pub inclination: String,
}

impl DummySpherical {
    pub fn clear(&mut self) {
        self.radius = String::new();
        self.azimuth = String::new();
        self.inclination = String::new();
    }

    pub fn content<'a>(&'a self) -> Element<'a, Message, Theme> {
        Column::new()
            .push(create_text_input(
                "radius",
                self.radius.as_str(),
                Message::SphericalRadiusChanged,
            ))
            .push(create_text_input(
                "azimuth",
                self.azimuth.as_str(),
                Message::SphericalAzimuthChanged,
            ))
            .push(create_text_input(
                "inclination",
                self.inclination.as_str(),
                Message::SphericalInclinationChanged,
            ))
            .into()
    }

    pub fn get_values_from(&mut self, cyl: &Spherical) {
        self.radius = cyl.radius.to_string();
        self.azimuth = cyl.azimuth.to_string();
        self.inclination = cyl.inclination.to_string();
    }

    pub fn set_values_for(&self, cyl: &mut Spherical) {
        cyl.radius = self.radius.parse::<f64>().unwrap_or(0.0);
        cyl.azimuth = self.azimuth.parse::<f64>().unwrap_or(0.0);
        cyl.inclination = self.inclination.parse::<f64>().unwrap_or(0.0);
    }

    pub fn to_spherical(&self) -> Spherical {
        Spherical::new(
            self.radius.parse::<f64>().unwrap_or(0.0),
            self.azimuth.parse::<f64>().unwrap_or(0.0),
            self.inclination.parse::<f64>().unwrap_or(0.0),
        )
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum GravityPickList {
    #[default]
    None,
    TwoBody,
    Constant,
}

impl GravityPickList {
    pub const ALL: [GravityPickList; 3] = [
        GravityPickList::None,
        GravityPickList::TwoBody,
        GravityPickList::Constant,
    ];
}

impl std::fmt::Display for GravityPickList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                GravityPickList::None => "None",
                GravityPickList::Constant => "Constant",
                GravityPickList::TwoBody => "TwoBody",
            }
        )
    }
}
#[derive(Debug, Default, Clone)]
pub struct DummyGravity {
    pub name: String,
    pub model: GravityPickList,
}

impl DummyGravity {
    pub fn clear(&mut self) {
        self.name = String::new();
        self.model = GravityPickList::None;
    }

    pub fn content<'a>(
        &'a self,
        constant_gravity: &'a DummyConstantGravity,
        two_body: &'a DummyTwoBodyGravity,
        two_body_custom: &'a DummyTwoBodyCustom,
    ) -> Element<'a, Message, Theme> {
        let content = Column::new()
            .push(create_text_input(
                "name",
                self.name.as_str(),
                Message::GravityNameChanged,
            ))
            .push(
                pick_list(
                    &GravityPickList::ALL[..],
                    Some(self.model),
                    Message::GravityModelSelected,
                )
                .width(Length::FillPortion(1)),
            );
        let content = match self.model {
            GravityPickList::None => content,
            GravityPickList::Constant => content.push(constant_gravity.content()),
            GravityPickList::TwoBody => content.push(two_body.content(two_body_custom)),
        };
        content.into()
    }

    pub fn get_values_from(
        &mut self,
        g: &Gravity,
        dummy_constant: &mut DummyConstantGravity,
        dummy_two_body: &mut DummyTwoBodyGravity,
        dummy_two_body_custom: &mut DummyTwoBodyCustom,
    ) {
        match g {
            Gravity::Constant(constant) => {
                dummy_constant.get_values_from(constant);
                self.model = GravityPickList::Constant;
            }
            Gravity::TwoBody(two_body) => {
                dummy_two_body.get_values_from(two_body, dummy_two_body_custom);
                self.model = GravityPickList::TwoBody;
            }
        }
    }

    pub fn to_gravity(
        &self,
        dummy_constant: &DummyConstantGravity,
        dummy_two_body: &DummyTwoBodyGravity,
        dummy_two_body_custom: &DummyTwoBodyCustom,
    ) -> MultibodyGravity {
        let name = self.name.clone();
        let gravity = match self.model {
            GravityPickList::None => Gravity::Constant(ConstantGravity::new(0.0, 0.0, 0.0)),
            GravityPickList::Constant => Gravity::Constant(dummy_constant.to_gravity()),
            GravityPickList::TwoBody => {
                Gravity::TwoBody(dummy_two_body.to_gravity(dummy_two_body_custom))
            }
        };
        MultibodyGravity::new(&name,gravity)
    }
}

#[derive(Debug, Default, Clone)]
pub struct DummyConstantGravity {
    pub x: String,
    pub y: String,
    pub z: String,
}

impl DummyConstantGravity {
    pub fn clear(&mut self) {
        self.x = String::new();
        self.y = String::new();
        self.z = String::new();
    }

    pub fn content(&self) -> Element<Message, Theme> {
        Column::new()
            .push(create_text_input(
                "x",
                self.x.as_str(),
                Message::ConstantGravityXChanged,
            ))
            .push(create_text_input(
                "y",
                self.y.as_str(),
                Message::ConstantGravityYChanged,
            ))
            .push(create_text_input(
                "z",
                self.z.as_str(),
                Message::ConstantGravityZChanged,
            ))
            .into()
    }

    pub fn get_values_from(&mut self, g: &ConstantGravity) {
        self.x = g.value[0].to_string();
        self.y = g.value[1].to_string();
        self.z = g.value[2].to_string();
    }

    pub fn to_gravity(&self) -> ConstantGravity {
        ConstantGravity::new(
            self.x.parse::<f64>().unwrap_or(0.0),
            self.y.parse::<f64>().unwrap_or(0.0),
            self.z.parse::<f64>().unwrap_or(0.0),
        )
    }
}

#[derive(Debug, Default, Clone, Copy, PartialEq)]
pub enum TwoBodyPickList {
    #[default]
    None,
    Custom,
    Earth,
    Moon,
    Sun,
    Mercury,
    Venus,
    Mars,
    Jupiter,
    Saturn,
    Uranus,
    Neptune,
    Pluto,
}

impl TwoBodyPickList {
    pub const ALL: [TwoBodyPickList; 13] = [
        TwoBodyPickList::None,
        TwoBodyPickList::Custom,
        TwoBodyPickList::Earth,
        TwoBodyPickList::Moon,
        TwoBodyPickList::Sun,
        TwoBodyPickList::Mercury,
        TwoBodyPickList::Venus,
        TwoBodyPickList::Mars,
        TwoBodyPickList::Jupiter,
        TwoBodyPickList::Saturn,
        TwoBodyPickList::Uranus,
        TwoBodyPickList::Neptune,
        TwoBodyPickList::Pluto,
    ];
}

impl std::fmt::Display for TwoBodyPickList {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                TwoBodyPickList::None => "None",
                TwoBodyPickList::Custom => "Custom",
                TwoBodyPickList::Earth => "Earth",
                TwoBodyPickList::Moon => "Moon",
                TwoBodyPickList::Sun => "Sun",
                TwoBodyPickList::Mercury => "Mercury",
                TwoBodyPickList::Venus => "Venus",
                TwoBodyPickList::Mars => "Mars",
                TwoBodyPickList::Jupiter => "Jupiter",
                TwoBodyPickList::Saturn => "Saturn",
                TwoBodyPickList::Uranus => "Uranus",
                TwoBodyPickList::Neptune => "Neptune",
                TwoBodyPickList::Pluto => "Pluto",
            }
        )
    }
}

#[derive(Debug, Default, Clone)]
pub struct DummyTwoBodyGravity {
    pub model: TwoBodyPickList,
}

impl DummyTwoBodyGravity {
    pub fn clear(&mut self) {
        self.model = TwoBodyPickList::None;
    }

    pub fn content<'a>(
        &'a self,
        two_body_custom: &'a DummyTwoBodyCustom,
    ) -> Element<'a, Message, Theme> {
        let content = Column::new().push(
            pick_list(
                &TwoBodyPickList::ALL[..],
                Some(self.model),
                Message::TwoBodyModelSelected,
            )
            .width(Length::FillPortion(1)),
        );
        let content = match self.model {
            TwoBodyPickList::Custom => content.push(two_body_custom.content()),
            _ => content,
        };

        content.into()
    }

    pub fn get_values_from(
        &mut self,
        g: &TwoBodyGravity,
        two_body_custom: &mut DummyTwoBodyCustom,
    ) {
        self.model = match g.mu {
            0.0 => TwoBodyPickList::None,
            gravity::EARTH => TwoBodyPickList::Earth,
            gravity::SUN => TwoBodyPickList::Sun,
            gravity::MOON => TwoBodyPickList::Moon,
            gravity::MERCURY => TwoBodyPickList::Mercury,
            gravity::VENUS => TwoBodyPickList::Venus,
            gravity::JUPITER => TwoBodyPickList::Jupiter,
            gravity::SATURN => TwoBodyPickList::Saturn,
            gravity::URANUS => TwoBodyPickList::Uranus,
            gravity::NEPTUNE => TwoBodyPickList::Neptune,
            gravity::PLUTO => TwoBodyPickList::Pluto,
            _ => {
                two_body_custom.mu = g.mu.to_string();
                TwoBodyPickList::Custom
            }
        };
    }

    pub fn to_gravity(&self, two_body_custom: &DummyTwoBodyCustom) -> TwoBodyGravity {
        match self.model {
            TwoBodyPickList::None => TwoBodyGravity::new(0.0),
            TwoBodyPickList::Earth => TwoBodyGravity::new(gravity::EARTH),
            TwoBodyPickList::Sun => TwoBodyGravity::new(gravity::SUN),
            TwoBodyPickList::Moon => TwoBodyGravity::new(gravity::MOON),
            TwoBodyPickList::Mercury => TwoBodyGravity::new(gravity::MERCURY),
            TwoBodyPickList::Venus => TwoBodyGravity::new(gravity::VENUS),
            TwoBodyPickList::Mars => TwoBodyGravity::new(gravity::MARS),
            TwoBodyPickList::Jupiter => TwoBodyGravity::new(gravity::JUPITER),
            TwoBodyPickList::Saturn => TwoBodyGravity::new(gravity::SATURN),
            TwoBodyPickList::Uranus => TwoBodyGravity::new(gravity::URANUS),
            TwoBodyPickList::Neptune => TwoBodyGravity::new(gravity::NEPTUNE),
            TwoBodyPickList::Pluto => TwoBodyGravity::new(gravity::PLUTO),
            TwoBodyPickList::Custom => {
                TwoBodyGravity::new(two_body_custom.mu.parse::<f64>().unwrap_or(0.0))
            }
        }
    }
}

#[derive(Debug, Default, Clone)]
pub struct DummyTwoBodyCustom {
    pub mu: String,
}

impl DummyTwoBodyCustom {
    pub fn clear(&mut self) {
        self.mu = String::new();
    }

    pub fn content(&self) -> Element<Message, Theme> {
        create_text_input("mu", self.mu.as_str(), Message::TwoBodyCustomMuChanged).into()
    }
}
