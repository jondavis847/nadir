use coordinate_systems::{
    cartesian::Cartesian, cylindrical::Cylindrical, spherical::Spherical, CoordinateSystem,
};

use rotations::{Rotation, RotationTrait};
use std::ops::Mul;

/// ROTATION IS ACTUALLY A TRANSFORM(PASSIVE ROTATION), NOT AN ACTIVE ROTATION
#[derive(Debug, Default, Copy, Clone)]
pub struct Transform {
    rotation: Rotation,
    translation: CoordinateSystem,
}

impl Transform {
    pub fn new(rotation: Rotation, translation: CoordinateSystem) -> Self {
        Self {
            rotation,
            translation,
        }
    }
}

impl Mul<Transform> for Transform {
    type Output = Self;

    fn mul(self, rhs: Transform) -> Transform {
        let new_rotation = self.rotation * rhs.rotation;
        let old_translation = Cartesian::from(self.translation).vec();
        let lhs_translation = Cartesian::from_vec(&rhs.rotation.transform(old_translation));
        let new_translation = match rhs.translation {
            CoordinateSystem::Cartesian(rhs_translation) => {
                CoordinateSystem::Cartesian(lhs_translation + rhs_translation)
            }
            CoordinateSystem::Cylindrical(rhs_translation) => {
                CoordinateSystem::Cylindrical(Cylindrical::from(lhs_translation) + rhs_translation)
            }
            CoordinateSystem::Spherical(rhs_translation) => {
                CoordinateSystem::Spherical(Spherical::from(lhs_translation) + rhs_translation)
            }
        };
        Transform::new(new_rotation, new_translation)
    }
}
