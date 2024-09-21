use super::*;
use nalgebra::{Matrix3, Vector3, Vector4};
use rand::prelude::*;
use serde::{Deserialize, Serialize};
use std::fmt;
use std::ops::{AddAssign, Mul, MulAssign, Neg};

/// A struct representing a quaternion for 3D rotations.
#[derive(Clone, Copy, PartialEq, Serialize, Deserialize)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub s: f64,
}

/// Errors that can occur when creating a `Quaternion`.
#[derive(Debug, Clone, Copy)]
pub enum QuaternionErrors {
    /// Occurs when the quaternion has zero magnitude.
    ZeroMagnitude,
}

impl Quaternion {
    /// Creates an identity quaternion.
    ///
    /// # Returns
    ///
    /// A `Quaternion` representing no rotation.
    pub const IDENTITY: Self = Self {
        x: 0.0,
        y: 0.0,
        z: 0.0,
        s: 1.0,
    };

    /// Creates a new normalized `Quaternion`.
    ///
    /// # Arguments
    ///
    /// * `x` - The x component of the quaternion.
    /// * `y` - The y component of the quaternion.
    /// * `z` - The z component of the quaternion.
    /// * `s` - The scalar component of the quaternion.
    ///
    /// # Returns
    ///
    /// A `Result` which is `Ok` containing a new `Quaternion` if the magnitude is non-zero,
    /// or an `Err` containing a `QuaternionErrors`.
    pub fn new(x: f64, y: f64, z: f64, s: f64) -> Self {
        Self { x, y, z, s }
    }

    // Dot product of two quaternions
    pub fn dot(&self, other: &Quaternion) -> f64 {
        self.x * other.x + self.y * other.y + self.z * other.z + self.s * other.s
    }

    // Function to compute the exponential of a quaternion
    pub fn exp(&self) -> Quaternion {
        let v = Vector3::new(self.x, self.y, self.z);
        let v_mag = v.magnitude();
        let cos_v = v_mag.cos();
        let sin_v = v_mag.sin();
        let u = v.normalize();

        let exp_s = self.s.exp();

        Quaternion {
            x: exp_s * u[0] * sin_v,
            y: exp_s * u[1] * sin_v,
            z: exp_s * u[2] * sin_v,
            s: exp_s * cos_v,
        }
    }

    pub fn log(&self) -> Quaternion {
        let theta = self.s.acos();

        let v = Vector3::new(self.x, self.y, self.z);
        let u = v.normalize();

        Quaternion {
            x: theta * u[0],
            y: theta * u[1],
            z: theta * u[2],
            s: 0.0,
        }
    }

    pub fn mag(&self) -> f64 {
        self.dot(self).sqrt()
    }

    pub fn powf(&self, pow: f64) -> Self {
        let theta = self.s.acos();
        let u = Vector3::new(self.x, self.y, self.z).normalize();

        let pow_theta = pow * theta;

        let s = pow_theta.cos();
        let v = u * pow_theta.sin();
        Quaternion::new(v[0], v[1], v[2], s)
    }

    pub fn normalize(&self) -> Self {
        let mag = self.dot(self).sqrt();
        if mag < f64::EPSILON {
            panic!("attemped to normalized Quaternion with zero mag")
        }
        Quaternion::new(self.x / mag, self.y / mag, self.z / mag, self.s / mag)
    }

    /// Creates a random quaternion.
    ///
    /// # Returns
    ///
    /// A random `Quaternion`.
    pub fn rand() -> Quaternion {
        let mut rng = thread_rng();
        let x = rng.gen_range(-1.0..1.0);
        let y = rng.gen_range(-1.0..1.0);
        let z = rng.gen_range(-1.0..1.0);
        let s = rng.gen_range(-1.0..1.0);

        Quaternion::new(x, y, z, s)
    }

    pub fn slerp(q1: &Quaternion, q2: &Quaternion, t: f64) -> Quaternion {
        let q1 = q1.normalize();
        let q2 = q2.normalize();

        // t is 0 - 1, where result is q1 when t is 0 and result is q2 when t is 1
        // Compute the cosine of the angle between the two quaternions
        let mut dot = q1.dot(&q2);

        // If the dot product is negative, slerp won't take the shorter path.
        // Note that q1 and -q1 are equivalent when the rotations are the same.
        let q2 = if dot < 0.0 {
            dot = -dot;
            Quaternion {
                x: -q2.x,
                y: -q2.y,
                z: -q2.z,
                s: -q2.s,
            }
        } else {
            q2
        };

        // If the quaternions are too close, use linear interpolation to avoid division by zero
        if dot > 0.9995 {
            let result = Quaternion {
                x: q1.x + t * (q2.x - q1.x),
                y: q1.y + t * (q2.y - q1.y),
                z: q1.z + t * (q2.z - q1.z),
                s: q1.s + t * (q2.s - q1.s),
            };
            return result;
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
    ) -> Quaternion {
        let q0 = q0.normalize();
        let q1 = q1.normalize();
        let q2 = q2.normalize();
        let q3 = q3.normalize();

        fn calculate_control_point(q0: Quaternion, q1: Quaternion, q2: Quaternion) -> Quaternion {
            let log_q = (q1 * q2.inv() * q0 * q1.inv()).log() * (-0.25);
            (log_q.exp() * q1).normalize()
        }

        // Calculate control points a and b
        let a = calculate_control_point(q0, q1, q2);
        let b = calculate_control_point(q1, q2, q3);

        let tmp1 = Quaternion::slerp(&q1, &q2, t);
        let tmp2 = Quaternion::slerp(&a, &b, t);

        Quaternion::slerp(&tmp1, &tmp2, 2.0 * t * (1.0 - t))
    }
}

impl RotationTrait for Quaternion {
    fn identity() -> Self {
        Self::IDENTITY
    }

    /// Computes the inverse of the quaternion.
    ///
    /// # Returns
    ///
    /// A new `Quaternion` representing the inverse.
    fn inv(&self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.s)
    }

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
    fn rotate(&self, v: Vector3<f64>) -> Vector3<f64> {
        let (q1, q2, q3, q4) = (self.x, self.y, self.z, self.s);

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
    fn transform(&self, v: Vector3<f64>) -> Vector3<f64> {
        let (q1, q2, q3, q4) = (self.x, self.y, self.z, self.s);

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
}

impl Default for Quaternion {
    /// Provides the default value for a quaternion.
    ///
    /// # Returns
    ///
    /// The identity quaternion.
    fn default() -> Self {
        Self::identity()
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
            self.s * rhs.x + self.x * rhs.s - self.y * rhs.z + self.z * rhs.y,
            self.s * rhs.y + self.y * rhs.s - self.z * rhs.x + self.x * rhs.z,
            self.s * rhs.z + self.z * rhs.s - self.x * rhs.y + self.y * rhs.x,
            self.s * rhs.s - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        )
    }
}

impl<'a> AddAssign<&'a Quaternion> for Quaternion {
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
        self.s += rhs.s;
        self.normalize(); // normalize here since this is last step of ODE
    }
}

impl Mul<f64> for Quaternion {
    type Output = Self;

    fn mul(self, rhs: f64) -> Self {
        Self::new(self.x * rhs, self.y * rhs, self.z * rhs, self.s * rhs)
    }
}

impl MulAssign<f64> for Quaternion {
    fn mul_assign(&mut self, rhs: f64) {
        self.x *= rhs;
        self.y *= rhs;
        self.z *= rhs;
        self.s *= rhs;
    }
}

impl Neg for Quaternion {
    type Output = Self;

    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z, -self.s)
    }
}

impl From<Vector4<f64>> for Quaternion {
    fn from(q: Vector4<f64>) -> Self {
        Self {
            x: q[0],
            y: q[1],
            z: q[2],
            s: q[3],
        }
    }
}

impl From<Rotation> for Quaternion {
    /// Converts a `Rotation` enum to a `Quaternion`.
    ///
    /// # Arguments
    ///
    /// * `rotation` - The rotation to be converted.
    ///
    /// # Returns
    ///
    /// The corresponding `Quaternion`.
    fn from(rotation: Rotation) -> Self {
        match rotation {
            Rotation::Quaternion(v) => v,
            Rotation::RotationMatrix(v) => Quaternion::from(v),
            Rotation::EulerAngles(v) => Quaternion::from(v),
        }
    }
}

impl From<EulerAngles> for Quaternion {
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
    fn from(euler_angles: EulerAngles) -> Self {
        let s = |v: f64| (v / 2.0).sin();
        let c = |v: f64| (v / 2.0).cos();

        let (phi, theta, psi) = (euler_angles.phi, euler_angles.theta, euler_angles.psi);

        let q = match euler_angles.sequence {
            EulerSequence::XYZ => Quaternion {
                x: s(phi) * c(theta) * c(psi) + c(phi) * s(theta) * s(psi),
                y: c(phi) * s(theta) * c(psi) - s(phi) * c(theta) * s(psi),
                z: c(phi) * c(theta) * s(psi) + s(phi) * s(theta) * c(psi),
                s: c(phi) * c(theta) * c(psi) - s(phi) * s(theta) * s(psi),
            },
            EulerSequence::XZY => Quaternion {
                x: s(phi) * c(theta) * c(psi) - c(phi) * s(theta) * s(psi),
                y: c(phi) * c(theta) * s(psi) - s(phi) * s(theta) * c(psi),
                z: c(phi) * s(theta) * c(psi) + s(phi) * c(theta) * s(psi),
                s: c(phi) * c(theta) * c(psi) + s(phi) * s(theta) * s(psi),
            },
            EulerSequence::YXZ => Quaternion {
                x: c(phi) * s(theta) * c(psi) + s(phi) * c(theta) * s(psi),
                y: s(phi) * c(theta) * c(psi) - c(phi) * s(theta) * s(psi),
                z: c(phi) * c(theta) * s(psi) - s(phi) * s(theta) * c(psi),
                s: c(phi) * c(theta) * c(psi) + s(phi) * s(theta) * s(psi),
            },
            EulerSequence::YZX => Quaternion {
                x: c(phi) * c(theta) * s(psi) + s(phi) * s(theta) * c(psi),
                y: s(phi) * c(theta) * c(psi) + c(phi) * s(theta) * s(psi),
                z: c(phi) * s(theta) * c(psi) - s(phi) * c(theta) * s(psi),
                s: c(phi) * c(theta) * c(psi) - s(phi) * s(theta) * s(psi),
            },
            EulerSequence::ZXY => Quaternion {
                x: c(phi) * s(theta) * c(psi) - s(phi) * c(theta) * s(psi),
                y: c(phi) * c(theta) * s(psi) + s(phi) * s(theta) * c(psi),
                z: c(phi) * s(theta) * s(psi) + s(phi) * c(theta) * c(psi),
                s: c(phi) * c(theta) * c(psi) - s(phi) * s(theta) * s(psi),
            },
            EulerSequence::ZYX => Quaternion {
                x: c(phi) * c(theta) * s(psi) - s(phi) * s(theta) * c(psi),
                y: c(phi) * s(theta) * c(psi) + s(phi) * c(theta) * s(psi),
                z: s(phi) * c(theta) * c(psi) - c(phi) * s(theta) * s(psi),
                s: c(phi) * c(theta) * c(psi) + s(phi) * s(theta) * s(psi),
            },
            EulerSequence::XYX => Quaternion {
                x: c(theta) * s(phi + psi),
                y: s(theta) * c(phi - psi),
                z: s(theta) * s(phi - psi),
                s: c(theta) * c(phi + psi),
            },
            EulerSequence::XZX => Quaternion {
                x: c(theta) * s(phi + psi),
                y: s(theta) * s(psi - phi),
                z: s(theta) * c(psi - phi),
                s: c(theta) * c(phi + psi),
            },
            EulerSequence::YXY => Quaternion {
                x: s(theta) * c(psi - phi),
                y: c(theta) * s(phi + psi),
                z: s(theta) * s(psi - phi),
                s: c(theta) * c(phi + psi),
            },
            EulerSequence::YZY => Quaternion {
                x: s(theta) * s(phi - psi),
                y: c(theta) * s(phi + psi),
                z: s(theta) * c(phi - psi),
                s: c(theta) * c(phi + psi),
            },
            EulerSequence::ZXZ => Quaternion {
                x: s(theta) * c(phi - psi),
                y: s(theta) * s(phi - psi),
                z: c(theta) * s(phi + psi),
                s: c(theta) * c(phi + psi),
            },
            EulerSequence::ZYZ => Quaternion {
                x: s(theta) * s(psi - phi),
                y: s(theta) * c(psi - phi),
                z: c(theta) * s(phi + psi),
                s: c(theta) * c(phi + psi),
            },
        };
        q.normalize() // normalize here since it's going to be a rotation
    }
}

impl From<RotationMatrix> for Quaternion {
    /// Converts a `RotationMatrix` to a `Quaternion`.
    ///
    /// # Arguments
    ///
    /// * `matrix` - The rotation matrix to be converted.
    ///
    /// # Returns
    ///
    /// The corresponding `Quaternion`.
    fn from(matrix: RotationMatrix) -> Self {
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

        let mut q = if trace > 0.0 {
            Quaternion::new(v[0], v[1], v[2], (trace + 1.0) / 2.0)
        } else {
            let mut m = m - (trace - 1.0) / 2.0 * Matrix3::identity();
            m = m + m.transpose();
            if m[(0, 0)] >= m[(1, 1)] && m[(0, 0)] >= m[(2, 2)] {
                Quaternion::new(m[(0, 0)], m[(1, 0)], m[(2, 0)], 2.0 * v[0])
            } else if m[(1, 1)] >= m[(2, 2)] {
                Quaternion::new(m[(0, 1)], m[(1, 1)], m[(2, 1)], 2.0 * v[1])
            } else {
                Quaternion::new(m[(0, 2)], m[(1, 2)], m[(2, 2)], 2.0 * v[2])
            }
        };
        if q.s < 0.0 {
            q = -q
        };
        q.normalize()
    }
}

impl fmt::Debug for Quaternion {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Quaternion ")?;
        writeln!(f, "   x: {: >10.6}", self.x)?;
        writeln!(f, "   y: {: >10.6}", self.y)?;
        writeln!(f, "   z: {: >10.6}", self.z)?;
        writeln!(f, "   s: {: >10.6}", self.s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx_eq::assert_approx_eq;
    use std::f64::consts::PI;
    const TOL: f64 = 1e-12;

    /// Test for quaternion normalization.
    #[test]
    fn test_quaternion_normalization() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0).normalize();

        assert_approx_eq!(q.x, 0.18257418583505536, TOL);
        assert_approx_eq!(q.y, 0.3651483716701107, TOL);
        assert_approx_eq!(q.z, 0.5477225575051661, TOL);
        assert_approx_eq!(q.s, 0.7302967433402214, TOL);
    }

    /// Test for quaternion inversion.
    #[test]
    fn test_quaternion_inv() {
        let quat = Quaternion::rand();
        let inv = quat.inv();

        assert_approx_eq!(inv.s, quat.s);
        assert_approx_eq!(inv.x, -quat.x);
        assert_approx_eq!(inv.y, -quat.y);
        assert_approx_eq!(inv.z, -quat.z);
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

        let result = Quaternion::from(m).normalize();

        assert_approx_eq!(result.x, 0.09229595564125728);
        assert_approx_eq!(result.y, -0.560985526796931);
        assert_approx_eq!(result.z, -0.4304593345768795);
        assert_approx_eq!(result.s, 0.701057384649978);
    }

    #[test]
    fn test_quaternion_from_xyz() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::XYZ);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.701057384649978);
        assert_approx_eq!(result.y, 0.092295955641257);
        assert_approx_eq!(result.z, 0.560985526796931);
        assert_approx_eq!(result.s, 0.430459334576879);
    }

    #[test]
    fn test_quaternion_from_xzy() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::XZY);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.430459334576879);
        assert_approx_eq!(result.y, -0.092295955641257);
        assert_approx_eq!(result.z, 0.560985526796931);
        assert_approx_eq!(result.s, 0.701057384649978);
    }

    #[test]
    fn test_quaternion_from_yxz() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::YXZ);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.560985526796931);
        assert_approx_eq!(result.y, 0.430459334576879);
        assert_approx_eq!(result.z, -0.092295955641257);
        assert_approx_eq!(result.s, 0.701057384649978);
    }

    #[test]
    fn test_quaternion_from_yzx() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::YZX);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.560985526796931);
        assert_approx_eq!(result.y, 0.701057384649978);
        assert_approx_eq!(result.z, 0.092295955641257);
        assert_approx_eq!(result.s, 0.430459334576879);
    }

    #[test]
    fn test_quaternion_from_zxy() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::ZXY);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.092295955641257);
        assert_approx_eq!(result.y, 0.560985526796931);
        assert_approx_eq!(result.z, 0.701057384649978);
        assert_approx_eq!(result.s, 0.430459334576879);
    }

    #[test]
    fn test_quaternion_from_zyx() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::ZYX);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, -0.092295955641257);
        assert_approx_eq!(result.y, 0.560985526796931);
        assert_approx_eq!(result.z, 0.430459334576879);
        assert_approx_eq!(result.s, 0.701057384649978);
    }

    #[test]
    fn test_quaternion_from_xyx() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::XYX);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.8001031451912654);
        assert_approx_eq!(result.y, 0.4619397662556433);
        assert_approx_eq!(result.z, 0.1913417161825449);
        assert_approx_eq!(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_xzx() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::XZX);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.8001031451912654);
        assert_approx_eq!(result.y, -0.1913417161825449);
        assert_approx_eq!(result.z, 0.4619397662556433);
        assert_approx_eq!(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_yxy() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::YXY);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.4619397662556433);
        assert_approx_eq!(result.y, 0.8001031451912654);
        assert_approx_eq!(result.z, -0.1913417161825449);
        assert_approx_eq!(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_yzy() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::YZY);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.1913417161825449);
        assert_approx_eq!(result.y, 0.8001031451912654);
        assert_approx_eq!(result.z, 0.4619397662556433);
        assert_approx_eq!(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_zxz() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::ZXZ);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, 0.4619397662556433);
        assert_approx_eq!(result.y, 0.1913417161825449);
        assert_approx_eq!(result.z, 0.8001031451912654);
        assert_approx_eq!(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_zyz() {
        let a = EulerAngles::new(PI / 2.0, PI / 3.0, PI / 4.0, EulerSequence::ZYZ);
        let result = Quaternion::from(a);

        assert_approx_eq!(result.x, -0.1913417161825449);
        assert_approx_eq!(result.y, 0.4619397662556433);
        assert_approx_eq!(result.z, 0.8001031451912654);
        assert_approx_eq!(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_transform_x() {
        let a = EulerAngles::new(PI / 4.0, 0.0, 0.0, EulerSequence::XYZ);
        let q = Quaternion::from(a);
        let v = Vector3::new(0.0, 1.0, 0.0);
        let result = q.transform(v);

        assert_approx_eq!(result[0], 0.0);
        assert_approx_eq!(result[1], 0.7071067811865475);
        assert_approx_eq!(result[2], -0.7071067811865476);
    }

    #[test]
    fn test_quaternion_rotate_x() {
        let a = EulerAngles::new(PI / 4.0, 0.0, 0.0, EulerSequence::XYZ);
        let q = Quaternion::from(a);
        let v = Vector3::new(0.0, 1.0, 0.0);
        let result = q.rotate(v);

        assert_approx_eq!(result[0], 0.0);
        assert_approx_eq!(result[1], 0.7071067811865475);
        assert_approx_eq!(result[2], 0.7071067811865476);
    }

    #[test]
    fn test_quaternion_transform_y() {
        let a = EulerAngles::new(0.0, PI / 4.0, 0.0, EulerSequence::XYZ);
        let q = Quaternion::from(a);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let result = q.transform(v);

        assert_approx_eq!(result[0], 0.7071067811865475);
        assert_approx_eq!(result[1], 0.0);
        assert_approx_eq!(result[2], 0.7071067811865476);
    }

    #[test]
    fn test_quaternion_rotate_y() {
        let a = EulerAngles::new(0.0, PI / 4.0, 0.0, EulerSequence::XYZ);
        let q = Quaternion::from(a);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let result = q.rotate(v);

        assert_approx_eq!(result[0], 0.7071067811865475);
        assert_approx_eq!(result[1], 0.0);
        assert_approx_eq!(result[2], -0.7071067811865476);
    }

    #[test]
    fn test_quaternion_transform_z() {
        let a = EulerAngles::new(0.0, 0.0, PI / 4.0, EulerSequence::XYZ);
        let q = Quaternion::from(a);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let result = q.transform(v);

        assert_approx_eq!(result[0], 0.7071067811865475);
        assert_approx_eq!(result[1], -0.7071067811865476);
        assert_approx_eq!(result[2], 0.0);
    }

    #[test]
    fn test_quaternion_rotate_z() {
        let a = EulerAngles::new(0.0, 0.0, PI / 4.0, EulerSequence::XYZ);
        let q = Quaternion::from(a);
        let v = Vector3::new(1.0, 0.0, 0.0);
        let result = q.rotate(v);

        assert_approx_eq!(result[0], 0.7071067811865475);
        assert_approx_eq!(result[1], 0.7071067811865476);
        assert_approx_eq!(result[2], 0.0);
    }

    #[test]
    fn test_quaternion_multiplication() {
        let q1 = Quaternion::new(
            0.7010573846499779,
            0.0922959556412572,
            0.560985526796931,
            0.43045933457687946,
        );
        let q2 = Quaternion::new(
            -0.41127872745152066,
            -0.4532968654326041,
            0.3615464744060406,
            0.7033177852005419,
        );

        let result = q2 * q1;

        assert_approx_eq!(result.x, 0.6036896179402393);
        assert_approx_eq!(result.y, -0.6143987193116093);
        assert_approx_eq!(result.z, 0.2703544012626616);
        assert_approx_eq!(result.s, 0.43009482282088674);
    }
}
