use crate::axis_angle::AxisAngleErrors;

use super::*;
use nalgebra::{Matrix3, Vector3, Vector4};
use rand::{prelude::*, rng};
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{AddAssign, Mul, MulAssign, Neg};
use thiserror::Error;
use uncertainty::{Dispersion, Uncertainty};

/// A struct representing a quaternion for 3D rotations.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub w: f64,
}

/// Errors that can occur when creating a `Quaternion`.
#[derive(Debug, Clone, Error, Copy)]
pub enum QuaternionErrors {
    #[error("{0}")]
    AxisAngleErrors(#[from] AxisAngleErrors),
    #[error("got zero magnitude quaternion")]
    ZeroMagnitude,
}

impl Quaternion {
    /// Creates an identity quaternion.
    ///
    /// # Returns
    ///
    /// A `Quaternion` representing no rotation.
    pub const IDENTITY: Self = Self { x: 0.0, y: 0.0, z: 0.0, w: 1.0 };

    /// Creates a new normalized `Quaternion`.
    ///
    /// # Arguments
    ///
    /// * `x` - The x component of the quaternion.
    /// * `y` - The y component of the quaternion.
    /// * `z` - The z component of the quaternion.
    /// * `w` - The scalar component of the quaternion.
    ///
    /// # Returns
    ///
    /// A `Result` which is `Ok` containing a new `Quaternion` if the magnitude is non-zero,
    /// or an `Err` containing a `QuaternionErrors`.
    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Self {
        Self { x, y, z, w }
    }

    // Dot product of two quaternions
    pub fn dot(&self, other: &Quaternion) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z + self.w * other.w
    }

    // Function to compute the exponential of a quaternion
    pub fn exp(&self) -> Quaternion {
        let v = Vector3::new(self.x, self.y, self.z);
        let v_mag = v.magnitude();
        let cos_v = v_mag.cos();
        let sin_v = v_mag.sin();
        let u = v.normalize();

        let exp_w = self
            .w
            .exp();

        Quaternion {
            x: exp_w * u[0] * sin_v,
            y: exp_w * u[1] * sin_v,
            z: exp_w * u[2] * sin_v,
            w: exp_w * cos_v,
        }
    }

    pub fn inv(&self) -> Quaternion {
        Quaternion::new(
            -self.x, -self.y, -self.z, self.w,
        )
    }
    pub fn log(&self) -> Quaternion {
        let theta = self
            .w
            .acos();

        let v = Vector3::new(self.x, self.y, self.z);
        let u = v.normalize();

        Quaternion { x: theta * u[0], y: theta * u[1], z: theta * u[2], w: 0.0 }
    }

    pub fn mag(&self) -> f64 {
        self.dot(self)
            .sqrt()
    }

    pub fn powf(&self, pow: f64) -> Self {
        let theta = self
            .w
            .acos();
        let u = Vector3::new(self.x, self.y, self.z).normalize();

        let pow_theta = pow * theta;

        let s = pow_theta.cos();
        let v = u * pow_theta.sin();
        Quaternion::new(v[0], v[1], v[2], s)
    }

    pub fn normalize(&self) -> Result<Self, QuaternionErrors> {
        let mag = self
            .dot(self)
            .sqrt();
        if mag < f64::EPSILON {
            return Err(QuaternionErrors::ZeroMagnitude);
        }
        Ok(Quaternion::new(
            self.x / mag,
            self.y / mag,
            self.z / mag,
            self.w / mag,
        ))
    }

    /// Creates a random quaternion.
    ///
    /// # Returns
    ///
    /// A random `Quaternion`.
    pub fn rand() -> Quaternion {
        let mut rng = rng();
        let x = rng.random_range(-1.0..1.0);
        let y = rng.random_range(-1.0..1.0);
        let z = rng.random_range(-1.0..1.0);
        let s = rng.random_range(-1.0..1.0);

        Quaternion::new(x, y, z, s)
    }

    pub fn slerp(q1: &Quaternion, q2: &Quaternion, t: f64) -> Result<Self, QuaternionErrors> {
        let q1 = q1.normalize()?;
        let q2 = q2.normalize()?;

        // t is 0 - 1, where result is q1 when t is 0 and result is q2 when t is 1
        // Compute the cosine of the angle between the two quaternions
        let mut dot = q1.dot(&q2);

        // If the dot product is negative, slerp won't take the shorter path.
        // Note that q1 and -q1 are equivalent when the rotations are the same.
        let q2 = if dot < 0.0 {
            dot = -dot;
            Quaternion { x: -q2.x, y: -q2.y, z: -q2.z, w: -q2.w }
        } else {
            q2
        };

        // If the quaternions are too close, use linear interpolation to avoid division by zero
        if dot > 0.9995 {
            let result = Quaternion {
                x: q1.x + t * (q2.x - q1.x),
                y: q1.y + t * (q2.y - q1.y),
                z: q1.z + t * (q2.z - q1.z),
                w: q1.w + t * (q2.w - q1.w),
            };
            return Ok(result);
        }

        ((q2 * q1.inv()).powf(t) * q1).normalize()
    }

    // Spherical Quadrangle interpolation
    pub fn squad(
        q0: Quaternion, // q[i-2]
        q1: Quaternion, // q[i-1]
        q2: Quaternion, // q[i]
        q3: Quaternion, // q[i+1]
        t: f64,
    ) -> Result<Self, QuaternionErrors> {
        let q0 = q0.normalize()?;
        let q1 = q1.normalize()?;
        let q2 = q2.normalize()?;
        let q3 = q3.normalize()?;

        fn calculate_control_point(
            q0: Quaternion,
            q1: Quaternion,
            q2: Quaternion,
        ) -> Result<Quaternion, QuaternionErrors> {
            let log_q = (q1 * q2.inv() * q0 * q1.inv()).log() * (-0.25);
            (log_q.exp() * q1).normalize()
        }

        // Calculate control points a and b
        let a = calculate_control_point(q0, q1, q2)?;
        let b = calculate_control_point(q1, q2, q3)?;

        let tmp1 = Quaternion::slerp(&q1, &q2, t)?;
        let tmp2 = Quaternion::slerp(&a, &b, t)?;

        Quaternion::slerp(
            &tmp1,
            &tmp2,
            2.0 * t * (1.0 - t),
        )
    }
}

#[derive(Clone, Copy, Debug, Serialize, Deserialize, Default)]
pub struct UnitQuaternion(pub Quaternion);
impl UnitQuaternion {
    pub const IDENTITY: Self = Self(Quaternion::IDENTITY);

    pub fn new(x: f64, y: f64, z: f64, w: f64) -> Result<Self, QuaternionErrors> {
        Ok(Self(
            Quaternion::new(x, y, z, w).normalize()?,
        ))
    }

    pub fn rand() -> Result<Self, QuaternionErrors> {
        Ok(Self(
            Quaternion::rand().normalize()?,
        ))
    }
}

impl TryFrom<&Quaternion> for UnitQuaternion {
    type Error = QuaternionErrors;
    fn try_from(value: &Quaternion) -> Result<Self, QuaternionErrors> {
        Ok(Self(value.normalize()?))
    }
}

impl From<&UnitQuaternion> for Quaternion {
    fn from(value: &UnitQuaternion) -> Self {
        value.0
    }
}

// We decide not to deref to quat since if a user doesnt import RotationTrait,
// inv() will be found for the Quaternion, returning Quaternion instead of UnitQuaternion

// impl Deref for UnitQuaternion {
//     type Target = Quaternion;
//     fn deref(&self) -> &Self::Target {
//         &self.0
//     }
// }

// impl DerefMut for UnitQuaternion {
//     fn deref_mut(&mut self) -> &mut Self::Target {
//         &mut self.0
//     }
// }

impl RotationTrait for UnitQuaternion {
    /// Rotates a vector by the quaternion.
    /// Follows the logic from Markley/Crassidis
    /// aka Active Rotation or "Alibi"
    /// See Markley/Crassidis section 2.4 and figure 2.2 for details
    /// # Arguments
    ///
    /// * `v` - The vector to be rotated.
    ///
    /// # Returns
    ///
    /// The rotated vector.
    fn rotate(&self, v: &Vector3<f64>) -> Vector3<f64> {
        let (q1, q2, q3, q4) = (
            self.0
                .x,
            self.0
                .y,
            self.0
                .z,
            self.0
                .w,
        );

        let out1 = (q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4) * v[0]
            + 2.0 * (q1 * q2 - q3 * q4) * v[1]
            + 2.0 * (q1 * q3 + q2 * q4) * v[2];

        let out2 = 2.0 * (q2 * q1 + q3 * q4) * v[0]
            + (-q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4) * v[1]
            + 2.0 * (q2 * q3 - q1 * q4) * v[2];

        let out3 = 2.0 * (q3 * q1 - q2 * q4) * v[0]
            + 2.0 * (q3 * q2 + q1 * q4) * v[1]
            + (-q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4) * v[2];

        Vector3::new(out1, out2, out3)
    }

    /// Transforms a vector by the quaternion.
    /// Follows the logic from Markley/Crassidis
    /// Section 2.9.3, equations 2.125 and 2.130
    /// aka Passive Rotation or "Alias"
    /// See Markley/Crassidis section 2.4 and figure 2.2 for details
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be transformed.
    ///
    /// # Returns
    ///
    /// The transformed vector.
    fn transform(&self, v: &Vector3<f64>) -> Vector3<f64> {
        let (q1, q2, q3, q4) = (
            self.0
                .x,
            self.0
                .y,
            self.0
                .z,
            self.0
                .w,
        );

        let out1 = (q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4) * v[0]
            + 2.0 * (q1 * q2 + q3 * q4) * v[1]
            + 2.0 * (q1 * q3 - q2 * q4) * v[2];

        let out2 = 2.0 * (q2 * q1 - q3 * q4) * v[0]
            + (-q1 * q1 + q2 * q2 - q3 * q3 + q4 * q4) * v[1]
            + 2.0 * (q2 * q3 + q1 * q4) * v[2];

        let out3 = 2.0 * (q3 * q1 + q2 * q4) * v[0]
            + 2.0 * (q3 * q2 - q1 * q4) * v[1]
            + (-q1 * q1 - q2 * q2 + q3 * q3 + q4 * q4) * v[2];

        Vector3::new(out1, out2, out3)
    }

    fn identity() -> Self {
        Self::IDENTITY
    }

    fn inv(&self) -> Self {
        UnitQuaternion(
            self.0
                .inv(),
        ) // no need to renormalize since just taking negative of a UnitQuaternion
    }
}

impl Default for Quaternion {
    /// Provides the default value for a quaternion.
    ///
    /// # Returns
    ///
    /// The identity quaternion.
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl Mul<Quaternion> for Quaternion {
    type Output = Self;

    /// Multiplies two quaternions.
    /// IMPORTANT: This follows from the "x" logic rather than the "dot" logic from Markley/Crassidis
    /// Successive multiplications act like DCMs so that a rotation from a2c is
    /// q_a2c = q_c2b * q_b2a
    /// This is different than most of the quaternion calculators out there.
    ///
    /// # Arguments
    ///
    /// * `rhs` - The right-hand side quaternion.
    ///
    /// # Returns
    ///
    /// The product of the two quaternions.
    fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.w * rhs.x + self.x * rhs.w - self.y * rhs.z + self.z * rhs.y,
            self.w * rhs.y + self.y * rhs.w - self.z * rhs.x + self.x * rhs.z,
            self.w * rhs.z + self.z * rhs.w - self.x * rhs.y + self.y * rhs.x,
            self.w * rhs.w - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        )
    }
}

impl Mul<UnitQuaternion> for UnitQuaternion {
    type Output = Self;
    fn mul(self, rhs: UnitQuaternion) -> Self::Output {
        Self(self.0 * rhs.0)
    }
}

impl AddAssign<&Quaternion> for Quaternion {
    /// Adds two quaternions
    /// NOTE: Quaternion addition for attitude is not necessarily defined.
    /// You would use quaternion multiplication for summing rotations.
    /// This is only used for adding quaternion derivatives
    /// to quaternion states in an ODE!            
    ///
    /// # Arguments
    ///
    /// * `rhs` - The right-hand side quaternion.
    ///
    /// # Returns
    ///
    /// The sum of the two quaternions.
    fn add_assign(&mut self, rhs: &Self) {
        self.x += rhs.x;
        self.y += rhs.y;
        self.z += rhs.z;
        self.w += rhs.w;
        self.normalize()
            .unwrap(); // normalize here since this is last step of ODE todo: error handling
    }
}

impl Mul<f64> for Quaternion {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self::new(
            self.x * rhs,
            self.y * rhs,
            self.z * rhs,
            self.w * rhs,
        )
    }
}

impl MulAssign<f64> for Quaternion {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
        self.w *= rhs;
    }
}

impl Neg for Quaternion {
    type Output = Self;

    fn neg(self) -> Self {
        Self::new(
            -self.x, -self.y, -self.z, -self.w,
        )
    }
}

impl Neg for UnitQuaternion {
    type Output = Self;

    fn neg(self) -> Self {
        // safe unwrap since this was alreaady a unit quaternion
        Self::new(
            -self
                .0
                .x,
            -self
                .0
                .y,
            -self
                .0
                .z,
            -self
                .0
                .w,
        )
        .unwrap()
    }
}

impl From<Vector4<f64>> for Quaternion {
    fn from(q: Vector4<f64>) -> Self {
        Self { x: q[0], y: q[1], z: q[2], w: q[3] }
    }
}

impl From<&Rotation> for UnitQuaternion {
    /// Converts a `Rotation` enum to a `Quaternion`.
    ///
    /// # Arguments
    ///
    /// * `rotation` - The rotation to be converted.
    ///
    /// # Returns
    ///
    /// The corresponding `Quaternion`.
    fn from(rotation: &Rotation) -> Self {
        match rotation {
            Rotation::Quaternion(v) => *v,
            Rotation::RotationMatrix(v) => UnitQuaternion::from(v),
            Rotation::EulerAngles(v) => UnitQuaternion::from(v),
        }
    }
}

impl From<&EulerAngles> for UnitQuaternion {
    /// Converts `EulerAngles` to a `Quaternion`.
    /// Reference: Markley & Crassidis, Fundamentals of Spacecraft Attitude Determination & Control
    ///
    /// # Arguments
    ///
    /// * `euler` - The Euler angles to be converted.
    ///
    /// # Returns
    ///
    /// The corresponding `Quaternion`.
    fn from(euler_angles: &EulerAngles) -> Self {
        let s = |v: f64| (v / 2.0).sin();
        let c = |v: f64| (v / 2.0).cos();

        let (phi, theta, psi) = (
            euler_angles.phi,
            euler_angles.theta,
            euler_angles.psi,
        );
        //unwraps should be safe due to trig math
        match euler_angles.sequence {
            EulerSequence::XYZ => UnitQuaternion::new(
                s(phi) * c(theta) * c(psi) + c(phi) * s(theta) * s(psi),
                c(phi) * s(theta) * c(psi) - s(phi) * c(theta) * s(psi),
                c(phi) * c(theta) * s(psi) + s(phi) * s(theta) * c(psi),
                c(phi) * c(theta) * c(psi) - s(phi) * s(theta) * s(psi),
            )
            .unwrap(),
            EulerSequence::XZY => UnitQuaternion::new(
                s(phi) * c(theta) * c(psi) - c(phi) * s(theta) * s(psi),
                c(phi) * c(theta) * s(psi) - s(phi) * s(theta) * c(psi),
                c(phi) * s(theta) * c(psi) + s(phi) * c(theta) * s(psi),
                c(phi) * c(theta) * c(psi) + s(phi) * s(theta) * s(psi),
            )
            .unwrap(),
            EulerSequence::YXZ => UnitQuaternion::new(
                c(phi) * s(theta) * c(psi) + s(phi) * c(theta) * s(psi),
                s(phi) * c(theta) * c(psi) - c(phi) * s(theta) * s(psi),
                c(phi) * c(theta) * s(psi) - s(phi) * s(theta) * c(psi),
                c(phi) * c(theta) * c(psi) + s(phi) * s(theta) * s(psi),
            )
            .unwrap(),
            EulerSequence::YZX => UnitQuaternion::new(
                c(phi) * c(theta) * s(psi) + s(phi) * s(theta) * c(psi),
                s(phi) * c(theta) * c(psi) + c(phi) * s(theta) * s(psi),
                c(phi) * s(theta) * c(psi) - s(phi) * c(theta) * s(psi),
                c(phi) * c(theta) * c(psi) - s(phi) * s(theta) * s(psi),
            )
            .unwrap(),
            EulerSequence::ZXY => UnitQuaternion::new(
                c(phi) * s(theta) * c(psi) - s(phi) * c(theta) * s(psi),
                c(phi) * c(theta) * s(psi) + s(phi) * s(theta) * c(psi),
                c(phi) * s(theta) * s(psi) + s(phi) * c(theta) * c(psi),
                c(phi) * c(theta) * c(psi) - s(phi) * s(theta) * s(psi),
            )
            .unwrap(),
            EulerSequence::ZYX => UnitQuaternion::new(
                c(phi) * c(theta) * s(psi) - s(phi) * s(theta) * c(psi),
                c(phi) * s(theta) * c(psi) + s(phi) * c(theta) * s(psi),
                s(phi) * c(theta) * c(psi) - c(phi) * s(theta) * s(psi),
                c(phi) * c(theta) * c(psi) + s(phi) * s(theta) * s(psi),
            )
            .unwrap(),
            EulerSequence::XYX => UnitQuaternion::new(
                c(theta) * s(phi + psi),
                s(theta) * c(phi - psi),
                s(theta) * s(phi - psi),
                c(theta) * c(phi + psi),
            )
            .unwrap(),
            EulerSequence::XZX => UnitQuaternion::new(
                c(theta) * s(phi + psi),
                s(theta) * s(psi - phi),
                s(theta) * c(psi - phi),
                c(theta) * c(phi + psi),
            )
            .unwrap(),
            EulerSequence::YXY => UnitQuaternion::new(
                s(theta) * c(psi - phi),
                c(theta) * s(phi + psi),
                s(theta) * s(psi - phi),
                c(theta) * c(phi + psi),
            )
            .unwrap(),
            EulerSequence::YZY => UnitQuaternion::new(
                s(theta) * s(phi - psi),
                c(theta) * s(phi + psi),
                s(theta) * c(phi - psi),
                c(theta) * c(phi + psi),
            )
            .unwrap(),
            EulerSequence::ZXZ => UnitQuaternion::new(
                s(theta) * c(phi - psi),
                s(theta) * s(phi - psi),
                c(theta) * s(phi + psi),
                c(theta) * c(phi + psi),
            )
            .unwrap(),
            EulerSequence::ZYZ => UnitQuaternion::new(
                s(theta) * s(psi - phi),
                s(theta) * c(psi - phi),
                c(theta) * s(phi + psi),
                c(theta) * c(phi + psi),
            )
            .unwrap(),
        }
    }
}

impl From<&RotationMatrix> for UnitQuaternion {
    /// Converts a `RotationMatrix` to a `Quaternion`.
    ///
    /// # Arguments
    ///
    /// * `matrix` - The rotation matrix to be converted.
    ///
    /// # Returns
    ///
    /// The corresponding `Quaternion`.
    fn from(matrix: &RotationMatrix) -> Self {
        let m = matrix.get_value();
        let trace = m[(0, 0)] + m[(1, 1)] + m[(2, 2)];

        // this is derived from markley
        /*
        if trace > 0.0 {
            let s = (trace + 1.0).sqrt() * 2.0;
            Quaternion {
                s: 0.25 * s,
                x: (m[(2, 1)] - m[(1, 2)]) / s,
                y: (m[(0, 2)] - m[(2, 0)]) / s,
                z: (m[(1, 0)] - m[(0, 1)]) / s,
            }
        } else if (m[(0, 0)] > m[(1, 1)]) && (m[(0, 0)] > m[(2, 2)]) {
            let s = (1.0_f64 + m[(0, 0)] - m[(1, 1)] - m[(2, 2)]).sqrt() * 2.0;
            Quaternion {
                s: (m[(2, 1)] - m[(1, 2)]) / s,
                x: 0.25 * s,
                y: (m[(0, 1)] + m[(1, 0)]) / s,
                z: (m[(0, 2)] + m[(2, 0)]) / s,
            }
        } else if m[(1, 1)] > m[(2, 2)] {
            let s = (1.0_f64 + m[(1, 1)] - m[(0, 0)] - m[(2, 2)]).sqrt() * 2.0;
            Quaternion {
                s: (m[(0, 2)] - m[(2, 0)]) / s,
                x: (m[(0, 1)] + m[(1, 0)]) / s,
                y: 0.25 * s,
                z: (m[(1, 2)] + m[(2, 1)]) / s,
            }
        } else {
            let s = (1.0_f64 + m[(2, 2)] - m[(0, 0)] - m[(1, 1)]).sqrt() * 2.0;
            Quaternion {
                s: (m[(1, 0)] - m[(0, 1)]) / s,
                x: (m[(0, 2)] + m[(2, 0)]) / s,
                y: (m[(1, 2)] + m[(2, 1)]) / s,
                z: 0.25 * s,
            }
        }
        */
        // this is just copy paste from featherstone
        let v = [
            (m[(2, 1)] - m[(1, 2)]) * 0.5,
            (m[(0, 2)] - m[(2, 0)]) * 0.5,
            (m[(1, 0)] - m[(0, 1)]) * 0.5,
        ];

        //unwraps should be safe due to validated quaternion math
        let mut q = if trace > 0.0 {
            UnitQuaternion::new(
                v[0],
                v[1],
                v[2],
                (trace + 1.0) / 2.0,
            )
            .unwrap()
        } else {
            let mut m = m - (trace - 1.0) / 2.0 * Matrix3::identity();
            m = m + m.transpose();
            if m[(0, 0)] >= m[(1, 1)] && m[(0, 0)] >= m[(2, 2)] {
                UnitQuaternion::new(
                    m[(0, 0)],
                    m[(1, 0)],
                    m[(2, 0)],
                    2.0 * v[0],
                )
                .unwrap()
            } else if m[(1, 1)] >= m[(2, 2)] {
                UnitQuaternion::new(
                    m[(0, 1)],
                    m[(1, 1)],
                    m[(2, 1)],
                    2.0 * v[1],
                )
                .unwrap()
            } else {
                UnitQuaternion::new(
                    m[(0, 2)],
                    m[(1, 2)],
                    m[(2, 2)],
                    2.0 * v[2],
                )
                .unwrap()
            }
        };
        if q.0
            .w
            < 0.0
        {
            q = -q
        };
        q
    }
}

impl From<&AxisAngle> for UnitQuaternion {
    fn from(axis_angle: &AxisAngle) -> Self {
        let half_angle = axis_angle.angle / 2.0;
        let s = half_angle.sin();
        let c = half_angle.cos();
        // unwrap shuold be safe due to trig math
        UnitQuaternion::new(
            s * axis_angle.axis[0],
            s * axis_angle.axis[1],
            s * axis_angle.axis[2],
            c,
        )
        .unwrap()
    }
}

impl fmt::Debug for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Quaternion ")?;
        writeln!(f, "   x: {: >10.6}", self.x)?;
        writeln!(f, "   y: {: >10.6}", self.y)?;
        writeln!(f, "   z: {: >10.6}", self.z)?;
        writeln!(f, "   w: {: >10.6}", self.w)
    }
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct UnitQuaternionBuilder {
    pub nominal: UnitQuaternion,
    dispersion: Option<Dispersion>, //angle in radians
}

impl UnitQuaternionBuilder {
    pub fn new(nominal: UnitQuaternion, dispersion: Option<Dispersion>) -> Self {
        Self { nominal, dispersion }
    }
}

impl Uncertainty for UnitQuaternionBuilder {
    type Output = UnitQuaternion;
    type Error = QuaternionErrors;
    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        if nominal {
            return Ok(self.nominal);
        }
        if let Some(dispersion) = &self.dispersion {
            let angle = dispersion.sample(rng);
            // uniformly sample the rotation axis
            let axis = Vector3::new(
                rng.random_range(-1.0..1.0),
                rng.random_range(-1.0..1.0),
                rng.random_range(-1.0..1.0),
            )
            .normalize();
            let error = UnitQuaternion::from(&AxisAngle::new(angle, axis)?);
            Ok(error * self.nominal)
        } else {
            Ok(self.nominal)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use std::f64::consts::PI;
    const TOL: f64 = 1e-12;

    /// Test for quaternion normalization.
    #[test]
    fn test_quaternion_normalization() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0)
            .normalize()
            .unwrap();
        let qn = UnitQuaternion::new(1.0, 2.0, 3.0, 4.0).unwrap();

        assert_abs_diff_eq!(
            q.x,
            0.18257418583505536,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            q.y,
            0.3651483716701107,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            q.z,
            0.5477225575051661,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            q.w,
            0.7302967433402214,
            epsilon = TOL
        );

        assert_abs_diff_eq!(
            qn.0.x,
            0.18257418583505536,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            qn.0.y,
            0.3651483716701107,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            qn.0.z,
            0.5477225575051661,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            qn.0.w,
            0.7302967433402214,
            epsilon = TOL
        );
    }

    /// Test for quaternion inversion.
    #[test]
    fn test_quaternion_inv() {
        let q = Quaternion::rand();
        let inv = q.inv();
        //TODO are these actually unit tests or do i need to use values?
        assert_abs_diff_eq!(inv.w, q.w, epsilon = TOL);
        assert_abs_diff_eq!(inv.x, -q.x, epsilon = TOL);
        assert_abs_diff_eq!(inv.y, -q.y, epsilon = TOL);
        assert_abs_diff_eq!(inv.z, -q.z, epsilon = TOL);

        let qn = UnitQuaternion::try_from(&q).unwrap();
        let inv = qn.inv();

        assert_abs_diff_eq!(
            inv.0
                .w,
            qn.0.w,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            inv.0
                .x,
            -qn.0
                .x,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            inv.0
                .y,
            -qn.0
                .y,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            inv.0
                .z,
            -qn.0
                .z,
            epsilon = TOL
        );
    }

    /// Test for quaternion multiplication.

    #[test]
    fn test_quaternion_from_rotation_matrix() {
        let m = RotationMatrix::new(
            0.0,
            0.5,
            -0.8660254037844386,
            -0.7071067811865476,
            0.6123724356957945,
            0.3535533905932738,
            0.7071067811865475,
            0.6123724356957946,
            0.35355339059327384,
        )
        .unwrap();

        let result = UnitQuaternion::from(&m);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.09229595564125728,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            -0.560985526796931,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            -0.4304593345768795,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.701057384649978,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_xyz() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::XYZ,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.701057384649978,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.092295955641257,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.560985526796931,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.430459334576879,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_xzy() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::XZY,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.430459334576879,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            -0.092295955641257,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.560985526796931,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.701057384649978,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_yxz() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::YXZ,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.560985526796931,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.430459334576879,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            -0.092295955641257,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.701057384649978,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_yzx() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::YZX,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.560985526796931,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.701057384649978,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.092295955641257,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.430459334576879,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_zxy() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::ZXY,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.092295955641257,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.560985526796931,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.701057384649978,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.430459334576879,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_zyx() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::ZYX,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            -0.092295955641257,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.560985526796931,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.430459334576879,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.701057384649978,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_xyx() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::XYX,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.8001031451912654,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.4619397662556433,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.1913417161825449,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.3314135740355918,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_xzx() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::XZX,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.8001031451912654,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            -0.1913417161825449,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.4619397662556433,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.3314135740355918,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_yxy() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::YXY,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.4619397662556433,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.8001031451912654,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            -0.1913417161825449,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.3314135740355918,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_yzy() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::YZY,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.1913417161825449,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.8001031451912654,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.4619397662556433,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.3314135740355918,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_zxz() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::ZXZ,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.4619397662556433,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.1913417161825449,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.8001031451912654,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.3314135740355918,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_from_zyz() {
        let a = EulerAngles::new(
            PI / 2.0,
            PI / 3.0,
            PI / 4.0,
            EulerSequence::ZYZ,
        );
        let result = UnitQuaternion::from(&a);

        assert_abs_diff_eq!(
            result
                .0
                .x,
            -0.1913417161825449,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            0.4619397662556433,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.8001031451912654,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.3314135740355918,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_transform_x() {
        let a = EulerAngles::new(
            PI / 4.0,
            0.0,
            0.0,
            EulerSequence::XYZ,
        );
        let q = UnitQuaternion::from(&a);
        let v = Vector3::new(0.0, 1.0, 0.0);
        let result = q.transform(&v);

        assert_abs_diff_eq!(result[0], 0.0, epsilon = TOL);
        assert_abs_diff_eq!(
            result[1],
            0.7071067811865475,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result[2],
            -0.7071067811865476,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_rotate_x() {
        let a = EulerAngles::new(
            PI / 4.0,
            0.0,
            0.0,
            EulerSequence::XYZ,
        );
        let q = UnitQuaternion::from(&a);
        let v = Vector3::new(0.0, 1.0, 0.0);
        let result = q.rotate(&v);

        assert_abs_diff_eq!(result[0], 0.0, epsilon = TOL);
        assert_abs_diff_eq!(
            result[1],
            0.7071067811865475,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result[2],
            0.7071067811865476,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_transform_y() {
        let a = EulerAngles::new(
            0.0,
            PI / 4.0,
            0.0,
            EulerSequence::XYZ,
        );
        let q = UnitQuaternion::from(&a);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let result = q.transform(&v);

        assert_abs_diff_eq!(
            result[0],
            0.7071067811865475,
            epsilon = TOL
        );
        assert_abs_diff_eq!(result[1], 0.0, epsilon = TOL);
        assert_abs_diff_eq!(
            result[2],
            0.7071067811865476,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_rotate_y() {
        let a = EulerAngles::new(
            0.0,
            PI / 4.0,
            0.0,
            EulerSequence::XYZ,
        );
        let q = UnitQuaternion::from(&a);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let result = q.rotate(&v);

        assert_abs_diff_eq!(
            result[0],
            0.7071067811865475,
            epsilon = TOL
        );
        assert_abs_diff_eq!(result[1], 0.0, epsilon = TOL);
        assert_abs_diff_eq!(
            result[2],
            -0.7071067811865476,
            epsilon = TOL
        );
    }

    #[test]
    fn test_quaternion_transform_z() {
        let a = EulerAngles::new(
            0.0,
            0.0,
            PI / 4.0,
            EulerSequence::XYZ,
        );
        let q = UnitQuaternion::from(&a);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let result = q.transform(&v);

        assert_abs_diff_eq!(
            result[0],
            0.7071067811865475,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result[1],
            -0.7071067811865476,
            epsilon = TOL
        );
        assert_abs_diff_eq!(result[2], 0.0, epsilon = TOL);
    }

    #[test]
    fn test_quaternion_rotate_z() {
        let a = EulerAngles::new(
            0.0,
            0.0,
            PI / 4.0,
            EulerSequence::XYZ,
        );
        let q = UnitQuaternion::from(&a);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let result = q.rotate(&v);

        assert_abs_diff_eq!(
            result[0],
            0.7071067811865475,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result[1],
            0.7071067811865476,
            epsilon = TOL
        );
        assert_abs_diff_eq!(result[2], 0.0, epsilon = TOL);
    }

    #[test]
    fn test_quaternion_multiplication() {
        let q1 = UnitQuaternion::new(
            0.7010573846499779,
            0.0922959556412572,
            0.560985526796931,
            0.43045933457687946,
        )
        .unwrap();
        let q2 = UnitQuaternion::new(
            -0.41127872745152066,
            -0.4532968654326041,
            0.3615464744060406,
            0.7033177852005419,
        )
        .unwrap();

        let result = q2 * q1;

        assert_abs_diff_eq!(
            result
                .0
                .x,
            0.6036896179402393,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .y,
            -0.6143987193116093,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .z,
            0.2703544012626616,
            epsilon = TOL
        );
        assert_abs_diff_eq!(
            result
                .0
                .w,
            0.43009482282088674,
            epsilon = TOL
        );
    }
}
