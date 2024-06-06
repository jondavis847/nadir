pub mod euler_angles;
pub mod quaternion;
pub mod rotation_matrix;

use euler_angles::EulerAngles;
use quaternion::Quaternion;
use rotation_matrix::RotationMatrix;

#[derive(Clone, Copy, Debug)]
pub enum Rotation {
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
