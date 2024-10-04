use rotations::Rotation;

use crate::gravity::Gravity;
use crate::geomag::GeoMagnetism;
use nalgebra::Vector3;
use serde::{Deserialize,Serialize};

#[derive(Debug,Clone,Deserialize,Serialize)]
pub struct Earth {
    position_icrf: Vector3<f64>,
    orientation_icrf: Rotation,
    gravity: Gravity,
    geomag: GeoMagnetism,
}