use super::{
    euler_angles::{EulerAngles, EulerSequence},
    Rotation,
    rotation_matrix::RotationMatrix,
};
use linear_algebra::Vector3;
use rand::prelude::*;
use std::ops::Mul;

#[derive(Debug, Clone, Copy)]
pub struct Quaternion {
    pub x: f64,
    pub y: f64,
    pub z: f64,
    pub s: f64,
}

#[derive(Debug, Clone, Copy)]
pub enum QuaternionErrors {
    ZeroMagnitude,
}

impl Quaternion {
    pub fn new(x: f64, y: f64, z: f64, s: f64) -> Result<Self, QuaternionErrors> {
        let mag_squared = x * x + y * y + z * z + s * s;

        if mag_squared < f64::EPSILON {
            return Err(QuaternionErrors::ZeroMagnitude);
        }

        let mag = mag_squared.sqrt();

        Ok(Self {
            x: x / mag,
            y: y / mag,
            z: z / mag,
            s: s / mag,
        })
    }

    pub fn identity() -> Self {
        Self::new(0.0, 0.0, 0.0, 1.0).unwrap()
    }

    pub fn rand() -> Quaternion {
        let mut rng = thread_rng();
        let x = rng.gen_range(-1.0..1.0);
        let y = rng.gen_range(-1.0..1.0);
        let z = rng.gen_range(-1.0..1.0);
        let s = rng.gen_range(-1.0..1.0);

        Quaternion::new(x, y, z, s).unwrap()
    }

    pub fn inv(&self) -> Self {
        Self::new(-self.x, -self.y, -self.z, self.s).unwrap()
    }

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
    fn default() -> Self {
        Self::identity()
    }
}

impl Mul<Quaternion> for Quaternion {
    type Output = Self;
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
    fn from(rotation: Rotation) -> Self {
        match rotation {
            Rotation::Quaternion(v) => v,
            Rotation::RotationMatrix(v) => Quaternion::from(v),
            Rotation::EulerAngles(v) => Quaternion::from(v),
        }                
    }
}

impl From<EulerAngles> for Quaternion {
    fn from(euler: EulerAngles) -> Self {
        let (phi, theta, psi) = (euler.x, euler.y, euler.z);
        let (c_phi, c_theta, c_psi) = ((phi / 2.0).cos(), (theta / 2.0).cos(), (psi / 2.0).cos());
        let (s_phi, s_theta, s_psi) = ((phi / 2.0).sin(), (theta / 2.0).sin(), (psi / 2.0).sin());

        match euler.sequence {
            EulerSequence::XYZ => Quaternion {
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
            },
            EulerSequence::ZYX => Quaternion {
                s: c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi - s_phi * s_theta * c_psi,
            },
            EulerSequence::XZY => Quaternion {
                s: c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
            },
            EulerSequence::YXZ => Quaternion {
                s: c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi - s_phi * s_theta * c_psi,
            },
            EulerSequence::YZX => Quaternion {
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi - s_phi * s_theta * c_psi,
            },
            EulerSequence::ZXY => Quaternion {
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
            },
            EulerSequence::XYX => Quaternion {
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi - s_phi * s_theta * c_psi,
            },
            EulerSequence::XZX => Quaternion {
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
            },
            EulerSequence::YXY => Quaternion {
                s: c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi - s_phi * s_theta * c_psi,
            },
            EulerSequence::YZY => Quaternion {
                s: c_phi * c_theta * c_psi + s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi - c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
            },
            EulerSequence::ZXZ => Quaternion {
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi + s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi - s_phi * s_theta * c_psi,
            },
            EulerSequence::ZYZ => Quaternion {
                s: c_phi * c_theta * c_psi - s_phi * s_theta * s_psi,
                x: s_phi * c_theta * c_psi + c_phi * s_theta * s_psi,
                y: c_phi * s_theta * c_psi - s_phi * c_theta * s_psi,
                z: c_phi * c_theta * s_psi + s_phi * s_theta * c_psi,
            },
        }
    }
}
impl From<RotationMatrix> for Quaternion {
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

    #[test]
    fn test_quaternion_normalization() {
        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0).unwrap();

        assert_eq!(q.x, 0.18257418583505536);
        assert_eq!(q.y, 0.3651483716701107);
        assert_eq!(q.z, 0.5477225575051661);
        assert_eq!(q.s, 0.7302967433402214);
    }

    #[test]
    fn test_quaternion_inv() {
        let quat = Quaternion::rand();
        let inv = quat.inv();

        assert_eq!(inv.s, quat.s);
        assert_eq!(inv.x, -quat.x);
        assert_eq!(inv.y, -quat.y);
        assert_eq!(inv.z, -quat.z);
    }

    #[test]
    fn test_quaternion_multiplication() {
        let quat1 = Quaternion::new(
            0.18119546436307749,
            0.4381103371317225,
            0.10015469662419728,
            0.8747551502773175,
        )
        .unwrap();
        let quat2 = Quaternion::new(
            0.4605004692970668,
            -0.13901506620501594,
            -0.7574522634418864,
            -0.44145237314115715,
        )
        .unwrap();
        let result = quat1 * quat2;

        assert_eq!(result.s, -0.3328369942072665);
        assert_eq!(result.x, 0.004911334761481288);
        assert_eq!(result.y, -0.13164079374848636);
        assert_eq!(result.z, -0.9337377123685223);
    }
}
