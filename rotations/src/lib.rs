mod quaternion;
mod rotation_matrix;

use quaternion::Quaternion;
use rotation_matrix::RotationMatrix;
use sim_value::SimValue;

#[derive(Clone, Copy, Debug)]
pub enum Rotation<T>
where
    T: SimValue,
{
    Quaternion(Quaternion<T>),
    RotationMatrix(RotationMatrix<T>),
}

impl<T> Default for Rotation<T>
where
    T: SimValue,
{
    fn default() -> Self {
        Rotation::Quaternion(Quaternion::identity())
    }
}