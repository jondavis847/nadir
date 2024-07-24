//use super::{quaternion::Quaternion, RotationTrait};
use super::*;
use linear_algebra::vector3::Vector3;

/// Enum representing different Euler angle sequences.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub enum EulerSequence {
    XYZ,
    XZY,
    YXZ,
    YZX,
    ZXY,
    #[default]
    ZYX,
    XYX,
    XZX,
    YXY,
    YZY,
    ZXZ,
    ZYZ,
}

// needed for iced pick_list
impl EulerSequence {
    pub const ALL: [EulerSequence; 12] = [
        EulerSequence::XYZ,
        EulerSequence::XZY,
        EulerSequence::YXZ,
        EulerSequence::YZX,
        EulerSequence::ZXY,
        EulerSequence::ZYX,
        EulerSequence::XYX,
        EulerSequence::XZX,
        EulerSequence::YXY,
        EulerSequence::YZY,
        EulerSequence::ZXZ,
        EulerSequence::ZYZ,
    ];
}

impl std::fmt::Display for EulerSequence {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "{}",
            match self {
                EulerSequence::XYZ => "XYZ",
                EulerSequence::XZY => "XZY",
                EulerSequence::YXZ => "YXZ",
                EulerSequence::YZX => "YZX",
                EulerSequence::ZXY => "ZXY",
                EulerSequence::ZYX => "ZYX",
                EulerSequence::XYX => "XYX",
                EulerSequence::XZX => "XZX",
                EulerSequence::YXY => "YXY",
                EulerSequence::YZY => "YZY",
                EulerSequence::ZXZ => "ZXZ",
                EulerSequence::ZYZ => "ZYZ",
            }
        )
    }
}

/// Struct representing Euler angles with a specific rotation sequence.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct EulerAngles {
    pub phi: f64,
    pub theta: f64,
    pub psi: f64,
    pub sequence: EulerSequence,
}

impl EulerAngles {
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
            sequence: EulerSequence::ZYX,
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
    pub fn new(phi: f64, theta: f64, psi: f64, sequence: EulerSequence) -> Self {
        Self {
            phi,
            theta,
            psi,
            sequence,
        }
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
        match self.sequence {
            EulerSequence::XYZ => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::ZYX)
            }
            EulerSequence::XZY => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::YZX)
            }
            EulerSequence::YXZ => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::ZXY)
            }
            EulerSequence::YZX => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::XZY)
            }
            EulerSequence::ZXY => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::YXZ)
            }
            EulerSequence::ZYX => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::XYZ)
            }
            EulerSequence::XYX => {
                EulerAngles::new(-self.phi, -self.theta, -self.psi, EulerSequence::XYX)
            }
            EulerSequence::XZX => {
                EulerAngles::new(-self.phi, -self.theta, -self.psi, EulerSequence::XZX)
            }
            EulerSequence::YXY => {
                EulerAngles::new(-self.phi, -self.theta, -self.psi, EulerSequence::YXY)
            }
            EulerSequence::YZY => {
                EulerAngles::new(-self.phi, -self.theta, -self.psi, EulerSequence::YZY)
            }
            EulerSequence::ZXZ => {
                EulerAngles::new(-self.phi, -self.theta, -self.psi, EulerSequence::ZXZ)
            }
            EulerSequence::ZYZ => {
                EulerAngles::new(-self.phi, -self.theta, -self.psi, EulerSequence::ZYZ)
            }
        }
    }

    /// Creates an identity `Angles` instance.
    ///
    /// # Returns
    ///
    /// A new `Angles` instance representing no rotation.
    fn identity() -> Self {
        EulerAngles::new(0.0, 0.0, 0.0, EulerSequence::ZYX)
    }
}
