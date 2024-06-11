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
    pub x: f64,
    pub y: f64,
    pub z: f64,
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
            x: v.e1,
            y: v.e2,
            z: v.e3,
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
    pub fn new(x: f64, y: f64, z: f64, sequence: EulerSequence) -> Self {
        Self { x, y, z, sequence }
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
            x: quat.x,
            y: quat.y,
            z: quat.z,
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
