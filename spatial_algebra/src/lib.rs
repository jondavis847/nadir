use linear_algebra::Vector3;
use rotations::rotation_matrix::RotationMatrix;
use std::ops::{Add, Mul};
use transforms::Transform;

pub struct Motion {
    pub rotation: Vector3,    //TODO should we force this as Cartesian?
    pub translation: Vector3, //TODO should we force this as Cartesian?
}

impl Motion {
    pub fn new(rotation: Vector3, translation: Vector3) -> Self {
        Self {
            rotation,
            translation,
        }
    }
}

pub struct Force {
    pub rotation: Vector3,    //TODO should we force this as Cartesian?
    pub translation: Vector3, //TODO should we force this as Cartesian?
}

impl Force {
    pub fn new(rotation: Vector3, translation: Vector3) -> Self {
        Self {
            rotation,
            translation,
        }
    }
}
pub struct SpatialTransform {
    value: Transform,
}

impl Add<Motion> for Motion {
    type Output = Motion;
    fn add(self, rhs: Motion) -> Motion {
        Motion::new(
            self.rotation + rhs.rotation,
            self.translation + rhs.rotation,
        )
    }
}

impl Mul<Motion> for SpatialTransform {
    type Output = Motion;
    fn mul(self, motion: Motion) -> Motion {
        let transform = self.value;
        let rotation_matrix = RotationMatrix::from(transform.rotation).value;
        let r_skew = transform.translation.vec().skew();

        let rotation = rotation_matrix * motion.rotation;
        let translation =
            rotation_matrix * motion.translation - rotation_matrix * r_skew * motion.rotation;
        Motion::new(rotation, translation)
    }
}

impl Add<Force> for Force {
    type Output = Force;
    fn add(self, rhs: Force) -> Force {
        Force::new(
            self.rotation + rhs.rotation,
            self.translation + rhs.rotation,
        )
    }
}


impl Mul<Force> for SpatialTransform {
    type Output = Force;
    fn mul(self, force: Force) -> Force {
        let transform = self.value;
        let rotation_matrix = RotationMatrix::from(transform.rotation).value;
        let r_skew = transform.translation.vec().skew();

        let rotation =
            rotation_matrix * force.rotation - rotation_matrix * r_skew * force.translation;
        let translation = rotation_matrix * force.translation;
        Force::new(rotation, translation)
    }
}
