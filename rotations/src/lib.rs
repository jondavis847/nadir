pub mod aligned_axes;
pub mod axis_angle;
pub mod euler_angles;
pub mod quaternion;
pub mod rotation_matrix;

use aligned_axes::AlignedAxes;
use axis_angle::AxisAngle;
use euler_angles::{EulerAngles, EulerSequence};
use nalgebra::Vector3;
use prelude::UnitQuaternion;
use serde::{Deserialize, Serialize};
use std::ops::Mul;

use quaternion::Quaternion;
use rotation_matrix::RotationMatrix;

pub mod prelude {
    pub use crate::aligned_axes::*;
    pub use crate::euler_angles::*;
    pub use crate::quaternion::*;
    pub use crate::rotation_matrix::*;
    pub use crate::{Rotation, RotationTrait};
}

/// Trait defining rotation and transformation operations.
pub trait RotationTrait {
    /// Rotates a vector by the rotation.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be rotated.
    ///
    /// # Returns
    ///
    /// The rotated vector.
    fn rotate(&self, v: &Vector3<f64>) -> Vector3<f64>;

    /// Transforms a vector by the rotation.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be transformed.
    ///
    /// # Returns
    ///
    /// The transformed vector.
    fn transform(&self, v: &Vector3<f64>) -> Vector3<f64>;

    fn inv(&self) -> Self;

    fn identity() -> Self;
}

/// Enum representing different types of rotations.
#[derive(Clone, Copy, Debug, Serialize, Deserialize)]
pub enum Rotation {
    EulerAngles(EulerAngles),
    Quaternion(UnitQuaternion),
    RotationMatrix(RotationMatrix),
}

impl Default for Rotation {
    /// Provides the default value for a rotation, which is an identity quaternion.
    ///
    /// # Returns
    ///
    /// The default `Rotation`, which is a quaternion representing no rotation.
    fn default() -> Self {
        Rotation::IDENTITY
    }
}

impl Rotation {
    pub const IDENTITY: Self = Rotation::Quaternion(UnitQuaternion::IDENTITY);
}

impl From<&UnitQuaternion> for Rotation {
    /// Converts Quaternion to a Rotation.
    ///
    /// # Arguments
    ///
    /// * `quaternion` - The quaternion.
    ///
    /// # Returns
    ///
    /// A new `Rotation` instance representing the quaternion.
    fn from(q: &UnitQuaternion) -> Self {
        Rotation::Quaternion(*q) // normalize since a rotation should be a unit quaternion
    }
}

impl From<&RotationMatrix> for Rotation {
    /// Converts RotationMatrix to a Rotation.
    ///
    /// # Arguments
    ///
    /// * `rotation_matrix` - The RotationMatrix.
    ///
    /// # Returns
    ///
    /// A new `Rotation` instance representing the RotationMatrix.
    fn from(rotation_matrix: &RotationMatrix) -> Self {
        Rotation::RotationMatrix(*rotation_matrix)
    }
}

impl From<&EulerAngles> for Rotation {
    /// Converts Euler angles to a rotation by converting to a quaternion.
    ///
    /// # Arguments
    ///
    /// * `euler` - The Euler angles to be converted.
    ///
    /// # Returns
    ///
    /// A new `Rotation` instance representing the converted quaternion.
    fn from(euler: &EulerAngles) -> Self {
        let q = UnitQuaternion::from(euler);
        Rotation::Quaternion(q)
    }
}

impl From<&AlignedAxes> for Rotation {
    /// Converts Aligned Axes to a rotation by converting to rotation matrix.
    /// TODO: Should we just go all the way to quaternion?
    ///
    /// # Arguments
    ///
    /// * `aligned_axes` - The Aligned Axes to be converted.
    ///
    /// # Returns
    ///
    /// A new `Rotation` instance representing the converted quaternion.
    fn from(aligned_axes: &AlignedAxes) -> Self {
        //let rotation_matrix = RotationMatrix::from(aligned_axes);
        //Rotation::RotationMatrix(rotation_matrix)
        Rotation::Quaternion(UnitQuaternion::from(&RotationMatrix::from(aligned_axes)))
    }
}

impl Mul<Rotation> for Rotation {
    type Output = Self;

    /// Multiplies two rotations.
    ///
    /// # Arguments
    ///
    /// * `rhs` - The right-hand side rotation to be multiplied.
    ///
    /// # Returns
    ///
    /// A new `Rotation` representing the product of the two rotations.
    fn mul(self, rhs: Rotation) -> Rotation {
        match (self, rhs) {
            (Rotation::RotationMatrix(lhs), Rotation::RotationMatrix(rhs)) => {
                Rotation::RotationMatrix(lhs * rhs)
            }
            (lhs, rhs) => {
                let q_lhs = UnitQuaternion::from(&lhs);
                let q_rhs = UnitQuaternion::from(&rhs);
                Rotation::Quaternion(q_lhs * q_rhs)
            }
        }
    }
}

impl RotationTrait for Rotation {
    /// Rotates a vector using the specified rotation.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be rotated.
    ///
    /// # Returns
    ///
    /// The rotated vector.
    fn rotate(&self, v: &Vector3<f64>) -> Vector3<f64> {
        match self {
            Rotation::EulerAngles(rotation) => rotation.rotate(v),
            Rotation::RotationMatrix(rotation) => rotation.rotate(v),
            Rotation::Quaternion(rotation) => rotation.rotate(v),
        }
    }

    /// Transforms a vector using the specified rotation.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be transformed.
    ///
    /// # Returns
    ///
    /// The transformed vector.
    fn transform(&self, v: &Vector3<f64>) -> Vector3<f64> {
        match self {
            Rotation::EulerAngles(rotation) => rotation.transform(v),
            Rotation::RotationMatrix(rotation) => rotation.transform(v),
            Rotation::Quaternion(rotation) => rotation.transform(v),
        }
    }

    fn inv(&self) -> Self {
        match self {
            Rotation::EulerAngles(rotation) => Rotation::EulerAngles(rotation.inv()),
            Rotation::RotationMatrix(rotation) => Rotation::RotationMatrix(rotation.inv()),
            Rotation::Quaternion(rotation) => Rotation::Quaternion(rotation.inv()),
        }
    }

    fn identity() -> Self {
        Rotation::IDENTITY
    }
}
