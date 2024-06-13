use coordinate_systems::cartesian::Cartesian;
use linear_algebra::Vector3;
use rotations::rotation_matrix::RotationMatrix;
use std::ops::Mul;
use transforms::Transform;

pub struct SpatialValue {
    pub rotation: Vector3,    //TODO should we force this as Cartesian?
    pub translation: Vector3, //TODO should we force this as Cartesian?
}

impl SpatialValue {
    pub fn new(rotation: Vector3, translation: Vector3) -> Self {
        Self {
            rotation,
            translation,
        }
    }
}

pub enum SpatialVector {
    Force(SpatialValue),
    Motion(SpatialValue),
}

pub struct SpatialTransform {
    value: Transform,
}

impl Mul<SpatialVector> for SpatialTransform {
    type Output = SpatialVector;
    fn mul(self, rhs: SpatialVector) -> SpatialVector {
        let transform = self.value;
        let rotation_matrix = RotationMatrix::from(transform.rotation).value;
        let r_skew = transform.translation.vec().skew();
        match rhs {
            SpatialVector::Motion(motion) => {
                let rotation = rotation_matrix * motion.rotation;
                let translation = rotation_matrix * motion.translation
                    - rotation_matrix * r_skew * motion.rotation;
                let spatial_value = SpatialValue::new(rotation, translation);
                SpatialVector::Motion(spatial_value)
            }
            SpatialVector::Force(force) => {
                let rotation =
                    rotation_matrix * force.rotation - rotation_matrix * r_skew * force.translation;
                let translation = rotation_matrix * force.translation;
                let spatial_value = SpatialValue::new(rotation, translation);
                SpatialVector::Force(spatial_value)
            }
        }
    }
}
