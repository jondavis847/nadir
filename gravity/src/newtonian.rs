use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use crate::{GravityErrors, GravityModel};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NewtonianGravity {
    pub mu: f64,
}

impl NewtonianGravity {
    pub fn new(mu: f64) -> Self {
        Self { mu }
    }
}

impl GravityModel for NewtonianGravity {
    fn calculate(&mut self, position: Vector3<f64>) -> Result<Vector3<f64>, GravityErrors> {
        let position_mag = position.magnitude();
        if position_mag < 0.1 {
            println!("WARNING! division by zero on gravity!");
        }
        Ok(-position * self.mu / position_mag.powi(3))
    }
}
