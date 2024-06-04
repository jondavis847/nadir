use crate::linear_algebra::Vector3;

#[derive(Debug, Copy, Clone)]
pub struct Cartesian {
    value: Vector3
}

impl Cartesian {
    pub fn new(x:f64, y:f64, z:f64) -> Self {
        Self {value: Vector3::new(x,y,z)}
    }
}

impl Default for Cartesian {
    fn default() -> Self {
        Self { value : Vector3::new(0.0,0.0,0.0)}
    }
}