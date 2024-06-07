use std::ops::Mul;

pub mod euler_angles;
pub mod quaternion;
pub mod rotation_matrix;

use coordinate_systems::{
    cartesian::Cartesian, cylindrical::Cylindrical, spherical::Spherical, CoordinateSystem,
};
use euler_angles::EulerAngles;
use linear_algebra::Vector3;
use quaternion::Quaternion;
use rotation_matrix::RotationMatrix;

#[derive(Clone, Copy, Debug)]
pub enum Rotation {
    EulerAngles(EulerAngles),
    Quaternion(Quaternion),
    RotationMatrix(RotationMatrix),
}

impl Default for Rotation {
    fn default() -> Self {
        Rotation::Quaternion(Quaternion::identity())
    }
}

impl From<EulerAngles> for Rotation {
    fn from(euler: EulerAngles) -> Self {
        let quaternion = Quaternion::from(euler);
        Rotation::Quaternion(quaternion)
    }
}

impl Mul<Rotation> for Rotation {
    type Output = Self;

    fn mul(self, rhs: Rotation) -> Rotation {
        match (self, rhs) {
            (Rotation::RotationMatrix(lhs), Rotation::RotationMatrix(rhs)) => {
                Rotation::RotationMatrix(lhs * rhs)
            }
            (lhs, rhs) => {
                let q_lhs = Quaternion::from(lhs);
                let q_rhs = Quaternion::from(rhs);
                Rotation::Quaternion(q_lhs * q_rhs)
            }
        }
    }
}

impl Mul<CoordinateSystem> for Rotation {
    type Output = CoordinateSystem;

    fn mul(self, rhs: CoordinateSystem) -> Self::Output {
        let old_cartesian = Cartesian::from(rhs);
        let old_translation = Vector3::new(old_cartesian.x, old_cartesian.y, old_cartesian.z);

        let new_translation = match self {
            Rotation::RotationMatrix(rotation_matrix) => rotation_matrix * old_translation,
            Rotation::Quaternion(quaternion) => quaternion.rotate(old_translation),
            Rotation::EulerAngles(euler_angles) => {
                let quaternion = Quaternion::from(euler_angles);
                quaternion.rotate(old_translation)
            }
        };

        let new_cartesian =
            Cartesian::new(new_translation.e1, new_translation.e2, new_translation.e3);
        match rhs {
            CoordinateSystem::Cartesian(_) => CoordinateSystem::Cartesian(new_cartesian),
            CoordinateSystem::Cylindrical(_) => {
                CoordinateSystem::Cylindrical(Cylindrical::from(new_cartesian))
            }
            CoordinateSystem::Spherical(_) => {
                CoordinateSystem::Spherical(Spherical::from(new_cartesian))
            }
        }
    }
}
