use super::{
    euler_angles::EulerAngles,
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

        // Ensure the scalar part is non-negative to enforce unique quaternions.
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

        match euler_angles {
            EulerAngles::XYZ(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: s(phi) * c(theta) * c(psi) + c(phi) * s(theta) * s(psi),
                    y: c(phi) * s(theta) * c(psi) - s(phi) * c(theta) * s(psi),
                    z: c(phi) * c(theta) * s(psi) + s(phi) * s(theta) * c(psi),
                    s: c(phi) * c(theta) * c(psi) - s(phi) * s(theta) * s(psi),
                }
            }
            EulerAngles::XZY(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: s(phi) * c(theta) * c(psi) - c(phi) * s(theta) * s(psi),
                    y: c(phi) * c(theta) * s(psi) - s(phi) * s(theta) * c(psi),
                    z: c(phi) * s(theta) * c(psi) + s(phi) * c(theta) * s(psi),
                    s: c(phi) * c(theta) * c(psi) + s(phi) * s(theta) * s(psi),
                }
            }
            EulerAngles::YXZ(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: c(phi) * s(theta) * c(psi) + s(phi) * c(theta) * s(psi),
                    y: s(phi) * c(theta) * c(psi) - c(phi) * s(theta) * s(psi),
                    z: c(phi) * c(theta) * s(psi) - s(phi) * s(theta) * c(psi),
                    s: c(phi) * c(theta) * c(psi) + s(phi) * s(theta) * s(psi),
                }
            }
            EulerAngles::YZX(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: c(phi) * c(theta) * s(psi) + s(phi) * s(theta) * c(psi),
                    y: s(phi) * c(theta) * c(psi) + c(phi) * s(theta) * s(psi),
                    z: c(phi) * s(theta) * c(psi) - s(phi) * c(theta) * s(psi),
                    s: c(phi) * c(theta) * c(psi) - s(phi) * s(theta) * s(psi),
                }
            }
            EulerAngles::ZXY(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: c(phi) * s(theta) * c(psi) - s(phi) * c(theta) * s(psi),
                    y: c(phi) * c(theta) * s(psi) + s(phi) * s(theta) * c(psi),
                    z: c(phi) * s(theta) * s(psi) + s(phi) * c(theta) * c(psi),
                    s: c(phi) * c(theta) * c(psi) - s(phi) * s(theta) * s(psi),
                }
            }
            EulerAngles::ZYX(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: c(phi) * c(theta) * s(psi) - s(phi) * s(theta) * c(psi),
                    y: c(phi) * s(theta) * c(psi) + s(phi) * c(theta) * s(psi),
                    z: s(phi) * c(theta) * c(psi) - c(phi) * s(theta) * s(psi),
                    s: c(phi) * c(theta) * c(psi) + s(phi) * s(theta) * s(psi),
                }
            }
            EulerAngles::XYX(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: c(theta) * s(phi + psi),
                    y: s(theta) * c(phi - psi),
                    z: s(theta) * s(phi - psi),
                    s: c(theta) * c(phi + psi),
                }
            }
            EulerAngles::XZX(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: c(theta) * s(phi + psi),
                    y: s(theta) * s(psi - phi),
                    z: s(theta) * c(psi - phi),
                    s: c(theta) * c(phi + psi),
                }
            }
            EulerAngles::YXY(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: s(theta) * c(psi - phi),
                    y: c(theta) * s(phi + psi),
                    z: s(theta) * s(psi - phi),
                    s: c(theta) * c(phi + psi),
                }
            }
            EulerAngles::YZY(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: s(theta) * s(phi - psi),
                    y: c(theta) * s(phi + psi),
                    z: s(theta) * c(phi - psi),
                    s: c(theta) * c(phi + psi),
                }
            }
            EulerAngles::ZXZ(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: s(theta) * c(phi - psi),
                    y: s(theta) * s(phi - psi),
                    z: c(theta) * s(phi + psi),
                    s: c(theta) * c(phi + psi),
                }
            }
            EulerAngles::ZYZ(angles) => {
                let (phi, theta, psi) = (angles.phi, angles.theta, angles.psi);
                Quaternion {
                    x: s(theta) * s(psi - phi),
                    y: s(theta) * c(psi - phi),
                    z: c(theta) * s(phi + psi),
                    s: c(theta) * c(phi + psi),
                }
            }
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
    use crate::euler_angles::Angles;
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
    fn test_quaternion_normalization() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0).unwrap();

        assert_close(q.x, 0.18257418583505536);
        assert_close(q.y, 0.3651483716701107);
        assert_close(q.z, 0.5477225575051661);
        assert_close(q.s, 0.7302967433402214);
    }

    /// Test for quaternion inversion.
    #[test]
    fn test_quaternion_inv() {
        let quat = Quaternion::rand();
        let inv = quat.inv();

        assert_close(inv.s, quat.s);
        assert_close(inv.x, -quat.x);
        assert_close(inv.y, -quat.y);
        assert_close(inv.z, -quat.z);
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

        assert_close(result.s, 0.3328369942072665);
        assert_close(result.x, -0.004911334761481288);
        assert_close(result.y, 0.13164079374848636);
        assert_close(result.z, 0.9337377123685223);
    }

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

        let result = Quaternion::from(m);

        assert_close(result.x, -0.09229595564125728);
        assert_close(result.y, 0.560985526796931);
        assert_close(result.z, 0.4304593345768795);
        assert_close(result.s, 0.701057384649978);
    }

    #[test]
    fn test_quaternion_from_xyz() {
        let a = EulerAngles::XYZ(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.701057384649978);
        assert_close(result.y, 0.092295955641257);
        assert_close(result.z, 0.560985526796931);
        assert_close(result.s, 0.430459334576879);
    }

    #[test]
    fn test_quaternion_from_xzy() {
        let a = EulerAngles::XZY(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.430459334576879);
        assert_close(result.y, -0.092295955641257);
        assert_close(result.z, 0.560985526796931);
        assert_close(result.s, 0.701057384649978);
    }

    #[test]
    fn test_quaternion_from_yxz() {
        let a = EulerAngles::YXZ(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.560985526796931);
        assert_close(result.y, 0.430459334576879);
        assert_close(result.z, -0.092295955641257);
        assert_close(result.s, 0.701057384649978);
    }

    #[test]
    fn test_quaternion_from_yzx() {
        let a = EulerAngles::YZX(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.560985526796931);
        assert_close(result.y, 0.701057384649978);
        assert_close(result.z, 0.092295955641257);
        assert_close(result.s, 0.430459334576879);
    }

    #[test]
    fn test_quaternion_from_zxy() {
        let a = EulerAngles::ZXY(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.092295955641257);
        assert_close(result.y, 0.560985526796931);
        assert_close(result.z, 0.701057384649978);
        assert_close(result.s, 0.430459334576879);
    }

    #[test]
    fn test_quaternion_from_zyx() {
        let a = EulerAngles::ZYX(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, -0.092295955641257);
        assert_close(result.y, 0.560985526796931);
        assert_close(result.z, 0.430459334576879);
        assert_close(result.s, 0.701057384649978);
    }

    #[test]
    fn test_quaternion_from_xyx() {
        let a = EulerAngles::XYX(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.8001031451912654);
        assert_close(result.y, 0.4619397662556433);
        assert_close(result.z, 0.1913417161825449);
        assert_close(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_xzx() {
        let a = EulerAngles::XZX(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.8001031451912654);
        assert_close(result.y, -0.1913417161825449);
        assert_close(result.z, 0.4619397662556433);
        assert_close(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_yxy() {
        let a = EulerAngles::YXY(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.4619397662556433);
        assert_close(result.y, 0.8001031451912654);
        assert_close(result.z, -0.1913417161825449);
        assert_close(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_yzy() {
        let a = EulerAngles::YZY(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.1913417161825449);
        assert_close(result.y, 0.8001031451912654);
        assert_close(result.z, 0.4619397662556433);
        assert_close(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_zxz() {
        let a = EulerAngles::ZXZ(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, 0.4619397662556433);
        assert_close(result.y, 0.1913417161825449);
        assert_close(result.z, 0.8001031451912654);
        assert_close(result.s, 0.3314135740355918);
    }

    #[test]
    fn test_quaternion_from_zyz() {
        let a = EulerAngles::ZYZ(Angles::new(PI / 2.0, PI / 3.0, PI / 4.0));
        let result = Quaternion::from(a);

        assert_close(result.x, -0.1913417161825449);
        assert_close(result.y, 0.4619397662556433);
        assert_close(result.z, 0.8001031451912654);
        assert_close(result.s, 0.3314135740355918);
    }
}
