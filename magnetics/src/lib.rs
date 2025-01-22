use dipole::{Dipole, DipoleErrors};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

pub mod dipole;
pub mod igrf;

use igrf::{Igrf, IgrfErrors};
use time::Time;

#[derive(Debug, Error)]
pub enum MagneticErrors {
    #[error("DipoleErrors: {0}")]
    Dipolerrors(#[from] DipoleErrors),
    #[error("IgrfErrors: {0}")]
    IgrfErrors(#[from] IgrfErrors),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Magnetics {
    Dipole(Dipole),
    Igrf(Igrf),
}

impl Magnetics {
    pub fn calculate(
        &mut self,
        r: &Vector3<f64>,
        epoch: &Time,
    ) -> Result<Vector3<f64>, MagneticErrors> {
        match self {
            Magnetics::Dipole(b) => match b.calculate(r) {
                Ok(b) => Ok(b),
                Err(e) => Err(e.into()),
            },
            Magnetics::Igrf(b) => match b.calculate(r, epoch) {
                Ok(b) => Ok(b),
                Err(e) => Err(e.into()),
            },
        }
    }
}
