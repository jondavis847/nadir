use std::ops::Mul;
pub mod euler_angles;
pub mod quaternion;
pub mod rotation_matrix;

use euler_angles::EulerAngles;
use linear_algebra::Vector3;
use quaternion::Quaternion;
use rotation_matrix::RotationMatrix;

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
    fn rotate(&self, v: Vector3) -> Vector3;

    /// Transforms a vector by the rotation.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be transformed.
    ///
    /// # Returns
    ///
    /// The transformed vector.
    fn transform(&self, v: Vector3) -> Vector3;
}

/// Enum representing different types of rotations.
#[derive(Clone, Copy, Debug)]
pub enum Rotation {
    EulerAngles(EulerAngles),
    Quaternion(Quaternion),
    RotationMatrix(RotationMatrix),
}

impl Default for Rotation {
    /// Provides the default value for a rotation, which is an identity quaternion.
    ///
    /// # Returns
    ///
    /// The default `Rotation`, which is a quaternion representing no rotation.
    fn default() -> Self {
        Rotation::Quaternion(Quaternion::identity())
    }
}

impl From<EulerAngles> for Rotation {
    /// Converts Euler angles to a rotation by converting to a quaternion.
    ///
    /// # Arguments
    ///
    /// * `euler` - The Euler angles to be converted.
    ///
    /// # Returns
    ///
    /// A new `Rotation` instance representing the converted quaternion.
    fn from(euler: EulerAngles) -> Self {
        let quaternion = Quaternion::from(euler);
        Rotation::Quaternion(quaternion)
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
                let q_lhs = Quaternion::from(lhs);
                let q_rhs = Quaternion::from(rhs);
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
    fn rotate(&self, v: Vector3) -> Vector3 {
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
    fn transform(&self, v: Vector3) -> Vector3 {
        match self {
            Rotation::EulerAngles(rotation) => rotation.transform(v),
            Rotation::RotationMatrix(rotation) => rotation.transform(v),
            Rotation::Quaternion(rotation) => rotation.transform(v),
        }
    }
}
