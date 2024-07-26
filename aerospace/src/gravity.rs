use nalgebra::Vector3;

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
    mu: f64,
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
