use crate::prelude::UnitQuaternion;
use nalgebra::Vector3;
use thiserror::Error;

use crate::RotationTrait;

#[derive(Debug, Error, Copy, Clone)]
pub enum AxisAngleErrors {
    #[error("magnitude of the axis is too small, should be normalizable to a magnitude of 1.0")]
    ZeroMagnitudeAxis,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AxisAngle {
    pub angle: f64,
    pub axis: Vector3<f64>,
}

impl AxisAngle {
    const IDENTITY: Self = Self { angle: 0.0, axis: Vector3::new(1.0, 0.0, 0.0) };

    pub fn new(angle: f64, axis: Vector3<f64>) -> Result<Self, AxisAngleErrors> {
        if axis.norm() < 1e-12 {
            return Err(AxisAngleErrors::ZeroMagnitudeAxis);
        }
        let axis = axis.normalize();
        Ok(Self { angle, axis })
    }
}

impl RotationTrait for AxisAngle {
    fn rotate(&self, v: &Vector3<f64>) -> Vector3<f64> {
        let quaternion = UnitQuaternion::from(self);
        quaternion.rotate(v)
    }

    fn transform(&self, v: &Vector3<f64>) -> Vector3<f64> {
        let quaternion = UnitQuaternion::from(self);
        quaternion.transform(v)
    }

    fn inv(&self) -> Self {
        Self { angle: -self.angle, axis: self.axis }
    }

    fn identity() -> Self {
        Self::IDENTITY
    }
}
