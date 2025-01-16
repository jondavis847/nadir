use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::{GravityErrors, GravityModel};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConstantGravity {
    pub g: Vector3<f64>,
}

impl ConstantGravity {
    pub fn new(g: Vector3<f64>) -> Self {
        Self { g }
    }
}

impl GravityModel for ConstantGravity {
    fn calculate(&mut self, _position: Vector3<f64>) -> Result<Vector3<f64>, GravityErrors> {
        Ok(self.g)
    }
}
