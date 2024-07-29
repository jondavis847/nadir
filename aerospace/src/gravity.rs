use nalgebra::Vector3;

pub const EARTH: f64 = 3.986004418e14;

pub const MOON: f64 = 4.9048695e12;

pub const SUN: f64 = 1.32712440018e20;

pub const MERCURY: f64 = 2.2032e13;

pub const VENUS: f64 = 3.24859e14;

pub const MARS: f64 = 4.282837e13;

pub const JUPITER: f64 = 1.26686534e17;

pub const SATURN: f64 = 3.7931187e16;

pub const URANUS: f64 = 5.793939e15;

pub const NEPTUNE: f64 = 6.836529e15;

pub const PLUTO: f64 = 8.71e11;

pub trait GravityTrait {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64>;
}

#[derive(Debug, Clone)]
pub enum Gravity {
    Constant(ConstantGravity),
    TwoBody(TwoBodyGravity),
}

#[derive(Debug, Clone)]
pub struct ConstantGravity {
    pub value: Vector3<f64>,
}
impl ConstantGravity {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            value: Vector3::new(x, y, z),
        }
    }
}

#[derive(Debug, Clone)]
pub struct TwoBodyGravity {
    pub mu: f64,
}

impl TwoBodyGravity {
    pub const EARTH: Self = Self { mu: EARTH };

    pub const MOON: Self = Self { mu: MOON };

    pub const SUN: Self = Self { mu: SUN };

    pub const MERCURY: Self = Self { mu: MERCURY };

    pub const VENUS: Self = Self { mu: VENUS };

    pub const MARS: Self = Self { mu: MARS };

    pub const JUPITER: Self = Self { mu: JUPITER };

    pub const SATURN: Self = Self { mu: SATURN };

    pub const URANUS: Self = Self { mu: URANUS };

    pub const NEPTUNE: Self = Self { mu: NEPTUNE };

    pub const PLUTO: Self = Self { mu: PLUTO };

    pub fn new(mu: f64) -> Self {
        Self { mu }
    }
}

impl GravityTrait for ConstantGravity {
    fn calculate(&self, _position: Vector3<f64>) -> Vector3<f64> {
        self.value
    }
}

impl GravityTrait for TwoBodyGravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        let position_mag = position.magnitude();
        position * self.mu / position_mag.powi(3)
    }
}

impl GravityTrait for Gravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        match self {
            Gravity::Constant(gravity) => gravity.calculate(position),
            Gravity::TwoBody(gravity) => gravity.calculate(position),
        }
    }
}
