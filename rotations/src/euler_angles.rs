//use super::{quaternion::Quaternion, RotationTrait};
use super::*;
use linear_algebra::vector3::Vector3;

/// Enum representing different Euler angle sequences.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum EulerAngles {
    XYZ(Angles),
    XZY(Angles),
    YXZ(Angles),
    YZX(Angles),
    ZXY(Angles),
    ZYX(Angles),
    XYX(Angles),
    XZX(Angles),
    YXY(Angles),
    YZY(Angles),
    ZXZ(Angles),
    ZYZ(Angles),
}

impl Default for EulerAngles {
    fn default() -> Self {
        Self::identity()
    }
}

/// Struct representing Euler angles with a specific rotation sequence.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Angles {
    pub phi: f64,
    pub theta: f64,
    pub psi: f64,
}

impl Angles {
    /// Creates a new `Angles` instance from a `Vector3` and a specified sequence.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector representing the Euler angles.    
    ///
    /// # Returns
    ///
    /// A new `Angles` instance.
    pub fn from_vec(v: &Vector3) -> Self {
        Self {
            phi: v.e1,
            theta: v.e2,
            psi: v.e3,
        }
    }

    /// Creates a new `Angles` instance with specified angles and sequence.
    ///
    /// # Arguments
    ///
    /// * `x` - The angle around the x-axis.
    /// * `y` - The angle around the y-axis.
    /// * `z` - The angle around the z-axis.    
    ///
    /// # Returns
    ///
    /// A new `Angles` instance.
    pub fn new(phi: f64, theta: f64, psi: f64) -> Self {
        Self { phi, theta, psi }
    }
}

//impl From<Quaternion> for EulerAngles::XYZ {
//TODO: Not easy, but probably not quite necessary for now
//Please see https://www.ncbi.nlm.nih.gov/pmc/articles/PMC9648712/ for a great algorithm for doing this for all sequences
//Just need to figure out how to properly do this in Rust
//The problem is I can't do From<Quaternion> for EulerAngles::XYZ
//A solution would be to make an EulerXYZ struct and impl From<Quaternion> for EulerXYZ
//but there's more important things to do for now
//}

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

    fn inv(&self) -> EulerAngles {
        match *self {
            EulerAngles::XYZ(angles) => {
                EulerAngles::ZYX(Angles::new(-angles.psi, -angles.theta, -angles.phi))
            }
            EulerAngles::XZY(angles) => {
                EulerAngles::YZX(Angles::new(-angles.psi, -angles.theta, -angles.phi))
            }
            EulerAngles::YXZ(angles) => {
                EulerAngles::ZXY(Angles::new(-angles.psi, -angles.theta, -angles.phi))
            }
            EulerAngles::YZX(angles) => {
                EulerAngles::XZY(Angles::new(-angles.psi, -angles.theta, -angles.phi))
            }
            EulerAngles::ZXY(angles) => {
                EulerAngles::YXZ(Angles::new(-angles.psi, -angles.theta, -angles.phi))
            }
            EulerAngles::ZYX(angles) => {
                EulerAngles::XYZ(Angles::new(-angles.psi, -angles.theta, -angles.phi))
            }
            EulerAngles::XYX(angles) => {
                EulerAngles::XYX(Angles::new(-angles.phi, -angles.theta, -angles.psi))
            }
            EulerAngles::XZX(angles) => {
                EulerAngles::XZX(Angles::new(-angles.phi, -angles.theta, -angles.psi))
            }
            EulerAngles::YXY(angles) => {
                EulerAngles::YXY(Angles::new(-angles.phi, -angles.theta, -angles.psi))
            }
            EulerAngles::YZY(angles) => {
                EulerAngles::YZY(Angles::new(-angles.phi, -angles.theta, -angles.psi))
            }
            EulerAngles::ZXZ(angles) => {
                EulerAngles::ZXZ(Angles::new(-angles.phi, -angles.theta, -angles.psi))
            }
            EulerAngles::ZYZ(angles) => {
                EulerAngles::ZYZ(Angles::new(-angles.phi, -angles.theta, -angles.psi))
            }
        }
    }

    /// Creates an identity `Angles` instance.
    ///
    /// # Returns
    ///
    /// A new `Angles` instance representing no rotation.
    fn identity() -> Self {
        EulerAngles::ZYX(Angles::new(0.0, 0.0, 0.0))
    }
}
