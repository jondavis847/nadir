use super::{
    euler_angles::{EulerAngles, EulerSequence},
    rotation_matrix::RotationMatrix,
    Rotation,
};
use linear_algebra::Vector3;
use rand::prelude::*;
use std::ops::Mul;

/// A struct representing a quaternion for 3D rotations.
#[derive(Debug, Clone, Copy)]
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
    pub fn new(x: f64, y: f64, z: f64, s: f64) -> Result<Self, QuaternionErrors> {
        let mut x = x;
        let mut y = y;
        let mut z = z;
        let mut s = s;

        let mag_squared = x * x + y * y + z * z + s * s;

        if mag_squared < f64::EPSILON {
            return Err(QuaternionErrors::ZeroMagnitude);
        }

        // Ensure the scalar part is non-negative for a consistent representation.
        if s < 0.0 {
            x = -x;
            y = -y;
            z = -z;
            s = -s;
        }

        let mag = mag_squared.sqrt();

        Ok(Self {
            x: x / mag,
            y: y / mag,
            z: z / mag,
            s: s / mag,
        })
    }

    /// Creates an identity quaternion.
    ///
    /// # Returns
    ///
    /// A `Quaternion` representing no rotation.
    pub fn identity() -> Self {
        Self::new(0.0, 0.0, 0.0, 1.0).unwrap()
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

        Quaternion::new(x, y, z, s).unwrap()
    }

    /// Computes the inverse of the quaternion.
    ///
    /// # Returns
    ///
    /// A new `Quaternion` representing the inverse.
    pub fn inv(&self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.s).unwrap()
    }

    /// Rotates a vector by the quaternion.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be rotated.
    ///
    /// # Returns
    ///
    /// The rotated vector.
    pub fn rotate(&self, v: Vector3) -> Vector3 {
        let v_mag = v.norm();
        if v_mag < f64::EPSILON {
            return Vector3::new(0.0, 0.0, 0.0);
        };
        let v_augmented = Self::new(v.e1, v.e2, v.e3, 0.0).unwrap();
        let inv_q = self.inv();
        let q_tmp = *self * v_augmented;
        let q_tmp = q_tmp * inv_q;
        Vector3::new(v_mag * q_tmp.x, v_mag * q_tmp.y, v_mag * q_tmp.z)
    }

    /// Transforms a vector by the quaternion.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be transformed.
    ///
    /// # Returns
    ///
    /// The transformed vector.
    pub fn transform(&self, v: Vector3) -> Vector3 {
        let v_mag = v.norm();
        if v_mag < f64::EPSILON {
            return Vector3::new(0.0, 0.0, 0.0);
        };
        let v_augmented = Self::new(v.e1, v.e2, v.e3, 0.0).unwrap();
        let inv_q = self.inv();
        let q_tmp = inv_q * v_augmented;
        let q_tmp = q_tmp * *self;
        Vector3::new(v_mag * q_tmp.x, v_mag * q_tmp.y, v_mag * q_tmp.z)
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
            self.s * rhs.x + self.x * rhs.s + self.y * rhs.z - self.z * rhs.y,
            self.s * rhs.y - self.x * rhs.z + self.y * rhs.s + self.z * rhs.x,
            self.s * rhs.z + self.x * rhs.y - self.y * rhs.x + self.z * rhs.s,
            self.s * rhs.s - self.x * rhs.x - self.y * rhs.y - self.z * rhs.z,
        )
        .unwrap()
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
    ///
    /// # Arguments
    ///
    /// * `euler` - The Euler angles to be converted.
    ///
    /// # Returns
    ///
    /// The corresponding `Quaternion`.
    fn from(euler: EulerAngles) -> Self {
        // phi is rotation about x
        // theta is rotation about y
        // psi is rotation about z
        let (phi, theta, psi) = (euler.x, euler.y, euler.z);
        let (c_phi, c_theta, c_psi) = ((phi / 2.0).cos(), (theta / 2.0).cos(), (psi / 2.0).cos());
        let (s_phi, s_theta, s_psi) = ((phi / 2.0).sin(), (theta / 2.0).sin(), (psi / 2.0).sin());

        match euler.sequence {
            EulerSequence::XYZ => Quaternion {                
                x: s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
            },
            
            EulerSequence::XZY => Quaternion {                
                x: s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
                y: c_phi * c_theta * s_psi - s_phi * s_theta * c_psi,
                z: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
                s: c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
            },
            EulerSequence::YXZ => Quaternion {                
                x: c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
                y: s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
                z: c_phi * c_theta * s_psi - s_phi * s_theta * c_psi,
                s: c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
            },
            EulerSequence::YZX => Quaternion {                
                x: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
                y: s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
                z: c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
            },
            EulerSequence::ZXY => Quaternion {                
                x: c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
                y: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
                z: c_phi * s_theta * s_psi + s_phi * c_theta * c_psi,
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
            },
            EulerSequence::ZYX => Quaternion {                
                x: c_phi * c_theta * s_psi - s_phi * s_theta * c_psi,
                y: c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
                z: s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
                s: c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
            },
            EulerSequence::XYX => Quaternion {                
                x: c_theta * (s_phi + s_psi),
                y: s_theta * (c_phi - c_psi),
                z: s_theta * (s_phi - s_psi),
                s: c_theta * (c_phi + c_psi),
            },
            EulerSequence::XZX => Quaternion {
                x: c_theta * (s_phi + s_psi),
                y: s_theta * (s_psi - s_phi),
                z: s_theta * (c_psi - c_phi),
                s: c_theta * (c_phi + c_psi),
            },
            EulerSequence::YXY => Quaternion {
                x: s_theta * (c_psi - c_phi),
                y: c_theta * (s_phi + s_psi),
                z: s_theta * (s_psi - s_phi),
                s: c_theta * (c_phi + c_psi),
            },
            EulerSequence::YZY => Quaternion {
                x: s_theta * (s_phi - s_psi),
                y: c_theta * (s_phi + c_psi),
                z: s_theta * (c_phi - c_psi),
                s: c_theta * (c_phi + c_psi),            },
            EulerSequence::ZXZ => Quaternion {
                x: s_theta * (c_phi - s_psi),
                y: s_theta * (s_phi - c_psi),
                z: c_theta * (s_phi + s_psi),
                s: c_theta * (c_phi + c_psi),
            },
            EulerSequence::ZYZ => Quaternion {
                x: s_theta * (s_psi - s_phi),
                y: s_theta * (c_psi - c_phi),
                z: c_theta * (s_phi + s_psi),
                s: c_theta * (c_phi + c_psi),
            },
        }
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
        let m = matrix.value;
        let trace = m.e11 + m.e22 + m.e33;

        if trace > 0.0 {
            let s = (trace + 1.0).sqrt() * 2.0;
            Quaternion {
                s: 0.25 * s,
                x: (m.e32 - m.e23) / s,
                y: (m.e13 - m.e31) / s,
                z: (m.e21 - m.e12) / s,
            }
        } else if (m.e11 > m.e22) && (m.e11 > m.e33) {
            let s = (1.0 + m.e11 - m.e22 - m.e33).sqrt() * 2.0;
            Quaternion {
                s: (m.e32 - m.e23) / s,
                x: 0.25 * s,
                y: (m.e12 + m.e21) / s,
                z: (m.e13 + m.e31) / s,
            }
        } else if m.e22 > m.e33 {
            let s = (1.0 + m.e22 - m.e11 - m.e33).sqrt() * 2.0;
            Quaternion {
                s: (m.e13 - m.e31) / s,
                x: (m.e12 + m.e21) / s,
                y: 0.25 * s,
                z: (m.e23 + m.e32) / s,
            }
        } else {
            let s = (1.0 + m.e33 - m.e11 - m.e22).sqrt() * 2.0;
            Quaternion {
                s: (m.e21 - m.e12) / s,
                x: (m.e13 + m.e31) / s,
                y: (m.e23 + m.e32) / s,
                z: 0.25 * s,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Test for quaternion normalization.
    #[test]
    fn test_quaternion_normalization() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0).unwrap();

        assert_eq!(q.x, 0.18257418583505536);
        assert_eq!(q.y, 0.3651483716701107);
        assert_eq!(q.z, 0.5477225575051661);
        assert_eq!(q.s, 0.7302967433402214);
    }

    /// Test for quaternion inversion.
    #[test]
    fn test_quaternion_inv() {
        let quat = Quaternion::rand();
        let inv = quat.inv();

        assert_eq!(inv.s, quat.s);
        assert_eq!(inv.x, -quat.x);
        assert_eq!(inv.y, -quat.y);
        assert_eq!(inv.z, -quat.z);
    }

    /// Test for quaternion multiplication.
    #[test]
    fn test_quaternion_multiplication() {
        let q1 = Quaternion::new(
            0.18119546436307749,
            0.4381103371317225,
            0.10015469662419728,
            0.8747551502773175,
        )
        .unwrap();
        let q2 = Quaternion::new(
            0.4605004692970668,
            -0.13901506620501594,
            -0.7574522634418864,
            -0.44145237314115715,
        )
        .unwrap();
        let result = q1 * q2;

        assert_eq!(result.s, -0.3328369942072665);
        assert_eq!(result.x, 0.004911334761481288);
        assert_eq!(result.y, -0.13164079374848636);
        assert_eq!(result.z, -0.9337377123685223);
    }

    #[test]
    fn test_quaternion_from_xyz() {
        let q1 = Quaternion::new(
            0.18119546436307749,
            0.4381103371317225,
            0.10015469662419728,
            0.8747551502773175,
        )
        .unwrap();
        let q2 = Quaternion::new(
            0.4605004692970668,
            -0.13901506620501594,
            -0.7574522634418864,
            -0.44145237314115715,
        )
        .unwrap();
        let result = q1 * q2;

        assert_eq!(result.s, -0.3328369942072665);
        assert_eq!(result.x, 0.004911334761481288);
        assert_eq!(result.y, -0.13164079374848636);
        assert_eq!(result.z, -0.9337377123685223);
    }
}
