use linear_algebra::vector3::Vector3;
use rotations::quaternion::Quaternion;
use spatial_algebra::Force;

#[derive(Debug, Clone, Copy, Default)]
pub struct BodyState {
    pub position: Vector3,
    pub velocity: Vector3,
    pub acceleration: Vector3,
    pub attitude: Quaternion,
    pub angular_rate: Vector3,
    pub external_force: Force,
}
