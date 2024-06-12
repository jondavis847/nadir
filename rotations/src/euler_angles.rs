use super::{quaternion::Quaternion, RotationTrait};
use linear_algebra::Vector3;

/// Enum representing different Euler angle sequences.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub enum EulerSequence {
    /// Default sequence ZYX.
    #[default]
    ZYX,
    XYZ,
    XZY,
    YXZ,
    YZX,
    ZXY,
    XYX,
    XZX,
    YXY,
    YZY,
    ZXZ,
    ZYZ,
}

/// Struct representing Euler angles with a specific rotation sequence.
#[derive(Debug, Clone, Copy, Default)]
pub struct EulerAngles {
    pub phi: f64,
    pub theta: f64,
    pub psi: f64,
    pub sequence: EulerSequence,
}

impl EulerAngles {
    /// Creates a new `EulerAngles` instance from a `Vector3` and a specified sequence.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector representing the Euler angles.
    /// * `sequence` - The sequence of rotations.
    ///
    /// # Returns
    ///
    /// A new `EulerAngles` instance.
    pub fn from_vec(v: &Vector3, sequence: EulerSequence) -> Self {
        Self {
            phi: v.e1,
            theta: v.e2,
            psi: v.e3,
            sequence: sequence,
        }
    }

    /// Creates a new `EulerAngles` instance with specified angles and sequence.
    ///
    /// # Arguments
    ///
    /// * `x` - The angle around the x-axis.
    /// * `y` - The angle around the y-axis.
    /// * `z` - The angle around the z-axis.
    /// * `sequence` - The sequence of rotations.
    ///
    /// # Returns
    ///
    /// A new `EulerAngles` instance.
    pub fn new(phi: f64, theta: f64, psi: f64, sequence: EulerSequence) -> Self {
        Self {
            phi,
            theta,
            psi,
            sequence,
        }
    }

    /// Creates an identity `EulerAngles` instance with default sequence ZYX.
    ///
    /// # Returns
    ///
    /// A new `EulerAngles` instance representing no rotation.
    pub fn identity() -> Self {
        Self::new(0.0, 0.0, 0.0, EulerSequence::ZYX)
    }
}

impl From<Quaternion> for EulerAngles {
    /// Converts a `Quaternion` into `EulerAngles` with the default sequence.
    ///
    /// # Arguments
    ///
    /// * `quat` - The quaternion to be converted.
    ///
    /// # Returns
    ///
    /// A new `EulerAngles` instance.
    fn from(quat: Quaternion) -> Self {
        EulerAngles {
            phi: quat.x,
            theta: quat.y,
            psi: quat.z,
            sequence: EulerSequence::default(),
        }
    }
}

impl RotationTrait for EulerAngles {
    /// Rotates a vector by the Euler angles.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be rotated.
    ///
    /// # Returns
    ///
    /// The rotated vector.
    fn rotate(&self, v: Vector3) -> Vector3 {
        let quat = Quaternion::from(*self);
        quat.rotate(v)
    }

    /// Transforms a vector by the Euler angles.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be transformed.
    ///
    /// # Returns
    ///
    /// The transformed vector.
    fn transform(&self, v: Vector3) -> Vector3 {
        let quat = Quaternion::from(*self);
        quat.transform(v)
    }
}

mod tests {
    use super::*;
    use std::f64::consts::PI;
    const TOL: f64 = 1e-12;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected,
            actual
        );
    }

    /// Test for quaternion normalization.
    #[test]
    fn test_xyz_from_quaternion() {
        let q = Quaternion::new(
            0.5,
            0.5,
            0.5,
            0.5,
        )
        .unwrap();
        let euler_angles = EulerAngles::from(q);

        assert_close(euler_angles.phi, 0.0);
        assert_close(euler_angles.theta, PI / 2.0);
        assert_close(euler_angles.psi, PI / 2.0);
        assert_eq!(euler_angles.sequence, EulerSequence::default());
    }
}
