use constant::ConstantGravity;
use egm::{EgmErrors, EgmGravity};
use nalgebra::Vector3;
use newtonian::NewtonianGravity;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub mod constant;
pub mod egm;
pub mod newtonian;

#[derive(Debug, Error)]
pub enum GravityErrors {
    #[error("EgmErrors: {0}")]
    EgmErrors(#[from] EgmErrors),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Gravity {
    Constant(ConstantGravity),
    Newtonian(NewtonianGravity),
    Egm(EgmGravity),
}

impl Gravity {
    pub fn calculate(&mut self, r: &Vector3<f64>) -> Result<Vector3<f64>, GravityErrors> {
        match self {
            Gravity::Constant(g) => g.calculate(r),
            Gravity::Newtonian(g) => g.calculate(r),
            Gravity::Egm(g) => g.calculate(r),
        }
    }
}

pub trait GravityModel {
    // input r is position vector in the central body's fixed (rotating) frame
    // returns gravitational acceleration perturbation in the central body's fixed (rotating) frame
    fn calculate(&mut self, r: &Vector3<f64>) -> Result<Vector3<f64>, GravityErrors>;
}
