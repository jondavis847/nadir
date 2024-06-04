use crate::coordinate_systems::cartesian::Cartesian;
use crate::rotations::Rotation;

#[derive(Debug, Default, Copy, Clone)]
pub struct Transform {
    translation: Cartesian,
    rotation: Rotation,
}