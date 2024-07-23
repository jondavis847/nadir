use geometry::cuboid::Cuboid;
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

#[derive(Debug, Default, Clone)]
pub struct Dummies {
    pub base: DummyBase,
    pub body: DummyBody,
    pub prismatic: DummyPrismatic,
    pub revolute: DummyRevolute,
    pub cuboid: DummyCuboid,
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
