use coordinate_systems::CoordinateSystem;
use rotations::Rotation;
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
        let rotation = self.rotation * rhs.rotation;
        let translation = rhs.rotation * self.translation + rhs.translation;
        Transform::new(rotation, translation)
    }
}
