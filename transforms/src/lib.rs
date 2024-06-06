use coordinate_systems::CoordinateSystem;
use rotations::Rotation;

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
