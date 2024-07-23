use geometry::cuboid::Cuboid;
use linear_algebra::matrix3::Matrix3;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use multibody::{
    base::Base,
    body::Body,
    joint::{
        prismatic::{Prismatic, PrismaticState},
        revolute::{Revolute, RevoluteState},
        Joint, JointParameters,
    },
    MultibodyTrait,
};
use rotations::{
    axes::Axis,
    euler_angles::{EulerAngles, EulerSequence},
    quaternion::Quaternion,
    rotation_matrix::RotationMatrix,
};

#[derive(Debug, Default, Clone)]
pub struct Dummies {
    pub base: DummyBase,
    pub body: DummyBody,
    pub prismatic: DummyPrismatic,
    pub revolute: DummyRevolute,
    pub cuboid: DummyCuboid,
    pub quaternion: DummyQuaternion,
    pub rotation_matrix: DummyRotationMatrix,
    pub euler_angles: DummyEulerAngles,
    pub aligned_axes: DummyAlignedAxes,
}

/// DummyComponents are like MultibodyComponents but with String fields
/// for editing in the text inputs rather than numeric values
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum DummyComponent {
    Base,
    Body,
    Revolute,
    Prismatic,
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

#[derive(Default, Debug, Clone, Copy)]
pub enum TransformPickList {
    #[default]
    Identity,
    Custom(TransformFields),
}

#[derive(Default, Debug, Clone, Copy)]
pub struct TransformFields {
    pub rotation: RotationPickList,
    pub translation: TranslationPickList,
}

#[derive(Default, Debug, Clone, Copy)]
pub enum RotationPickList {
    #[default]
    Identity,
    AlignedAxes,
    EulerAngles,
    Quaternion,
    RotationMatrix,
}

#[derive(Default, Debug, Clone, Copy)]
pub enum TranslationPickList {
    #[default]
    Zero,
    Cartesian,
    Cylindrical,
    Spherical,
}

#[derive(Default, Debug, Clone)]
pub struct DummyRevolute {
    pub constant_force: String,
    pub damping: String,
    pub name: String,
    pub omega: String,
    pub spring_constant: String,
    pub theta: String,
    pub inner_transform: TransformPickList,
    pub outer_transform: TransformPickList,
}

impl DummyRevolute {
    pub fn clear(&mut self) {
        self.name = String::new();
        self.theta = String::new();
        self.omega = String::new();
        self.spring_constant = String::new();
        self.damping = String::new();
        self.constant_force = String::new();
        self.inner_transform = TransformPickList::Identity;
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

    pub fn get_values_from(&mut self, rotation: &RotationMatrix) {
        self.e11 = rotation.0.e11.to_string();
        self.e12 = rotation.0.e12.to_string();
        self.e13 = rotation.0.e13.to_string();
        self.e21 = rotation.0.e21.to_string();
        self.e22 = rotation.0.e22.to_string();
        self.e23 = rotation.0.e23.to_string();
        self.e31 = rotation.0.e31.to_string();
        self.e32 = rotation.0.e32.to_string();
        self.e33 = rotation.0.e33.to_string();
    }

    pub fn set_values_for(&self, rotation: &mut RotationMatrix) {
        rotation.0.e11 = self.e11.parse::<f64>().unwrap_or(1.0);
        rotation.0.e12 = self.e12.parse::<f64>().unwrap_or(0.0);
        rotation.0.e13 = self.e13.parse::<f64>().unwrap_or(0.0);
        rotation.0.e21 = self.e21.parse::<f64>().unwrap_or(0.0);
        rotation.0.e22 = self.e22.parse::<f64>().unwrap_or(1.0);
        rotation.0.e23 = self.e23.parse::<f64>().unwrap_or(0.0);
        rotation.0.e31 = self.e31.parse::<f64>().unwrap_or(0.0);
        rotation.0.e32 = self.e32.parse::<f64>().unwrap_or(0.0);
        rotation.0.e33 = self.e33.parse::<f64>().unwrap_or(1.0);
    }

    pub fn to_rotation_matrix(&self) -> RotationMatrix {
        let mut rotation = RotationMatrix(Matrix3::IDENTITY);
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

    pub fn get_values_from(&mut self, euler_angles: &EulerAngles) {
        self.phi = euler_angles.phi.to_string();
        self.theta = euler_angles.theta.to_string();
        self.psi = euler_angles.psi.to_string();
        self.sequence = euler_angles.sequence;
    }

    pub fn set_values_for(&self, euler_angles: &mut EulerAngles) {
        euler_angles.phi = self.phi.parse::<f64>().unwrap();
        euler_angles.theta = self.theta.parse::<f64>().unwrap();
        euler_angles.psi = self.psi.parse::<f64>().unwrap();
        euler_angles.sequence = self.sequence;
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
