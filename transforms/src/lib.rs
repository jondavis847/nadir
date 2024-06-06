use coordinate_systems::CoordinateSystem;
use rotations::Rotation;

#[derive(Debug, Default, Copy, Clone)]
pub struct Transform {
    translation: CoordinateSystem,
    rotation: Rotation,
}
