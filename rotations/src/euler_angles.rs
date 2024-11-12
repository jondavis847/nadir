//use super::{quaternion::Quaternion, RotationTrait};
use super::*;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

/// Enum representing different Euler angle sequences.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Serialize, Deserialize)]
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
#[derive(Debug, Clone, Copy, Default, PartialEq, Serialize, Deserialize)]
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
    pub fn from_vec(v: &Vector3<f64>) -> Self {
        Self {
            phi: v[0],
            theta: v[1],
            psi: v[2],
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
    fn rotate(&self, v: Vector3<f64>) -> Vector3<f64> {
        let quat = Quaternion::from(self);
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
    fn transform(&self, v: Vector3<f64>) -> Vector3<f64> {
        let quat = Quaternion::from(self);
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
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::XYX)
            }
            EulerSequence::XZX => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::XZX)
            }
            EulerSequence::YXY => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::YXY)
            }
            EulerSequence::YZY => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::YZY)
            }
            EulerSequence::ZXZ => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::ZXZ)
            }
            EulerSequence::ZYZ => {
                EulerAngles::new(-self.psi, -self.theta, -self.phi, EulerSequence::ZYZ)
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

impl From<&Quaternion> for EulerAngles {
    fn from(q: &Quaternion) -> Self {
        let q = if q.s < 0.0 { -(*q) } else { *q };

        let q = q.normalize();

        // we assume ZYX rotation
        let w = q.s;
        let x = q.x;
        let y = q.y;
        let z = q.z;

        // Pitch Y (θ)
        let theta = (2.0 * (w * y - z * x)).asin();

        // Check for gimbal lock
        if theta.abs() > std::f64::consts::FRAC_PI_2 - 1e-6 {
            // Gimbal lock occurs, set roll to 0 and compute yaw directly
            let phi = 0.0; // Roll Z (φ) can be set to zero or inferred differently
            let psi = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (x * x + y * y));

            if theta > 0.0 {
                // Positive gimbal lock (Pitch = +90°)
                Self::new(phi, std::f64::consts::FRAC_PI_2, psi, EulerSequence::ZYX)
            } else {
                // Negative gimbal lock (Pitch = -90°)
                Self::new(phi, -std::f64::consts::FRAC_PI_2, psi, EulerSequence::ZYX)
            }
        } else {
            // No gimbal lock, calculate roll and yaw normally
            // Roll Z (φ)
            let phi = (2.0 * (w * x + y * z)).atan2(1.0 - 2.0 * (x * x + y * y));

            // Yaw X (ψ)
            let psi = (2.0 * (w * z + x * y)).atan2(1.0 - 2.0 * (y * y + z * z));

            Self::new(phi, theta, psi, EulerSequence::ZYX)
        }
    }
}
