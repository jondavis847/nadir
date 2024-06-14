use linear_algebra::Vector3;
use mass_properties::MassProperties;
use rotations::rotation_matrix::RotationMatrix;
use std::ops::{Add, Mul};
use transforms::Transform;

#[derive(Clone, Copy, Debug, Default)]
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
    /// Featherstone 2.33
    pub fn cross(self, rhs: Motion) -> Motion {
        let rotation = self.rotation.cross(rhs.rotation);
        let translation =
            self.rotation.cross(rhs.translation) + self.translation.cross(rhs.rotation);
        Motion::new(rotation, translation)
    }
}

#[derive(Clone, Copy, Debug, Default)]
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

    ///Featherstone 2.34
    pub fn cross(self, rhs: Force) -> Force {
        let rotation = self.rotation.cross(rhs.rotation) + self.translation.cross(rhs.translation);
        let translation = self.rotation.cross(rhs.translation);
        Force::new(rotation, translation)
    }
}

// we only need this as wrapper on Transform, other wise we cant impl Mul<Motion> for Transform since it's not in this crate :(
#[derive(Clone, Copy, Debug, Default)]
pub struct SpatialTransform {
    value: Transform,
}

impl SpatialTransform {
    #[inline]
    pub fn inv(&self) -> SpatialTransform {
        SpatialTransform::from(self.value.inv())
    }
}

impl From<Transform> for SpatialTransform {
    #[inline]
    fn from(value: Transform) -> SpatialTransform {
        SpatialTransform { value }
    }
}

impl Add<Motion> for Motion {
    type Output = Motion;
    #[inline]
    fn add(self, rhs: Motion) -> Motion {
        Motion::new(
            self.rotation + rhs.rotation,
            self.translation + rhs.translation,
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
    #[inline]
    fn add(self, rhs: Force) -> Force {
        Force::new(
            self.rotation + rhs.rotation,
            self.translation + rhs.translation,
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

impl Mul<SpatialTransform> for SpatialTransform {
    type Output = SpatialTransform;
    #[inline]
    fn mul(self, rhs: SpatialTransform) -> SpatialTransform {
        SpatialTransform::from(self.value * rhs.value)
    }
}

struct SpatialInertia {
    value: MassProperties,
}

impl From<MassProperties> for SpatialInertia {
    fn from(value: MassProperties) -> SpatialInertia {
        SpatialInertia { value }
    }
}
