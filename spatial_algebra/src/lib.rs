use linear_algebra::{matrix3::Matrix3, matrix6::Matrix6, vector3::Vector3, vector6::Vector6};
use mass_properties::MassProperties;
use rotations::rotation_matrix::RotationMatrix;
use std::ops::{Add, Mul, Sub};
use transforms::Transform;

#[derive(Clone, Copy, Debug, Default)]
pub struct SpatialVector {
    pub rotation: Vector3,    //TODO should we force this as Cartesian?
    pub translation: Vector3, //TODO should we force this as Cartesian?
}

impl SpatialVector {
    pub fn new(rotation: Vector3, translation: Vector3) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub fn vector(&self) -> Vector6 {
        Vector6::from_2vector3(self.rotation, self.translation)
    }
    ///Featherstone 2.34
    pub fn cross_force(self, rhs: SpatialVector) -> SpatialVector {
        let lhs_rotation = self.rotation;
        let lhs_translation = self.translation;
        let rhs_rotation = rhs.rotation;
        let rhs_translation = rhs.translation;

        let new_rotation =
            lhs_rotation.cross(rhs_rotation) + lhs_translation.cross(rhs_translation);
        let new_translation = lhs_rotation.cross(rhs_translation);
        SpatialVector::new(new_rotation, new_translation)
    }
    /// Featherstone 2.33
    pub fn cross_motion(self, rhs: SpatialVector) -> SpatialVector {
        let lhs_rotation = self.rotation;
        let lhs_translation = self.translation;
        let rhs_rotation = rhs.rotation;
        let rhs_translation = rhs.translation;

        let new_rotation = lhs_rotation.cross(rhs_rotation);
        let new_translation =
            lhs_rotation.cross(rhs_translation) + lhs_translation.cross(rhs_rotation);
        SpatialVector::new(new_rotation, new_translation)
    }
}

impl From<Vector6> for SpatialVector {
    fn from(v: Vector6) -> SpatialVector {
        let rotation = Vector3::new(v.e1, v.e2, v.e3);
        let translation = Vector3::new(v.e4, v.e5, v.e6);
        SpatialVector::new(rotation, translation)
    }
}

impl Add<SpatialVector> for SpatialVector {
    type Output = Self;
    #[inline]
    fn add(self, rhs: Self) -> Self {
        Self::new(
            self.rotation + rhs.rotation,
            self.translation + rhs.translation,
        )
    }
}

impl Sub<SpatialVector> for SpatialVector {
    type Output = Self;
    #[inline]
    fn sub(self, rhs: Self) -> Self {
        Self::new(
            self.rotation - rhs.rotation,
            self.translation - rhs.translation,
        )
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct MotionVector(SpatialVector);

impl MotionVector {
    pub fn vector(&self) -> Vector6 {
        self.0.vector()
    }

    pub fn cross_motion(self, rhs: MotionVector) -> MotionVector {
        MotionVector(self.0.cross_motion(rhs.0))
    }

    pub fn cross_force(self, rhs: ForceVector) -> ForceVector {
        ForceVector(self.0.cross_force(rhs.0))
    }
}

impl Add<MotionVector> for MotionVector {
    type Output = MotionVector;
    #[inline]
    fn add(self, rhs: MotionVector) -> MotionVector {
        MotionVector(self.0 + rhs.0)
    }
}

impl Sub<MotionVector> for MotionVector {
    type Output = MotionVector;
    #[inline]
    fn sub(self, rhs: MotionVector) -> MotionVector {
        MotionVector(self.0 - rhs.0)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Velocity(MotionVector);

impl Velocity {
    pub fn vector(&self) -> Vector6 {
        self.0.vector()
    }

    pub fn cross_motion(self, rhs: Velocity) -> Velocity {
        Velocity(self.0.cross_motion(rhs.0))
    }

    pub fn cross_force(self, rhs: Momentum) -> Force {
        Force(self.0.cross_force(rhs.0))
    }
}
impl From<MotionVector> for Velocity {
    fn from(motion: MotionVector) -> Self {
        Self(motion)
    }
}

impl Add<Velocity> for Velocity {
    type Output = Velocity;
    #[inline]
    fn add(self, rhs: Velocity) -> Velocity {
        Velocity(self.0 + rhs.0)
    }
}

impl Sub<Velocity> for Velocity {
    type Output = Velocity;
    #[inline]
    fn sub(self, rhs: Velocity) -> Velocity {
        Velocity(self.0 - rhs.0)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Acceleration(MotionVector);
impl From<MotionVector> for Acceleration {
    fn from(motion: MotionVector) -> Self {
        Self(motion)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ForceVector(SpatialVector);

impl ForceVector {}

impl Add<ForceVector> for ForceVector {
    type Output = ForceVector;
    #[inline]
    fn add(self, rhs: ForceVector) -> ForceVector {
        ForceVector(self.0 + rhs.0)
    }
}

impl Sub<ForceVector> for ForceVector {
    type Output = ForceVector;
    #[inline]
    fn sub(self, rhs: ForceVector) -> ForceVector {
        ForceVector(self.0 - rhs.0)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Momentum(ForceVector);

impl From<ForceVector> for Momentum {
    fn from(force: ForceVector) -> Self {
        Self(force)
    }
}

impl Add<Momentum> for Momentum {
    type Output = Momentum;
    #[inline]
    fn add(self, rhs: Momentum) -> Momentum {
        Momentum(self.0 + rhs.0)
    }
}

impl Sub<Momentum> for Momentum {
    type Output = Momentum;
    #[inline]
    fn sub(self, rhs: Momentum) -> Momentum {
        Momentum(self.0 - rhs.0)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct Force(ForceVector);

impl From<ForceVector> for Force {
    fn from(force: ForceVector) -> Self {
        Self(force)
    }
}

impl Add<Force> for Force {
    type Output = Force;
    #[inline]
    fn add(self, rhs: Force) -> Force {
        Force(self.0 + rhs.0)
    }
}

impl Sub<Force> for Force {
    type Output = Force;
    #[inline]
    fn sub(self, rhs: Force) -> Force {
        Force(self.0 - rhs.0)
    }
}

// we only need this as wrapper on Transform, other wise we cant impl Mul<Motion> for Transform since it's not in this crate :(
#[derive(Clone, Copy, Debug, Default)]
pub struct SpatialTransform(Transform);

impl SpatialTransform {
    #[inline]
    pub fn inv(&self) -> SpatialTransform {
        SpatialTransform::from(self.0.inv())
    }
}

impl From<Transform> for SpatialTransform {
    #[inline]
    fn from(value: Transform) -> Self {
        Self(value)
    }
}

impl Mul<MotionVector> for SpatialTransform {
    type Output = MotionVector;
    fn mul(self, motion: MotionVector) -> MotionVector {
        let transform = self.0;
        let rotation_matrix = RotationMatrix::from(transform.rotation).get_value();
        let r_skew = transform.translation.vec().skew();

        let rotation = rotation_matrix * motion.0.rotation;
        let translation =
            rotation_matrix * motion.0.translation - rotation_matrix * r_skew * motion.0.rotation;
        MotionVector(SpatialVector::new(rotation, translation))
    }
}

impl Mul<Velocity> for SpatialTransform {
    type Output = Velocity;
    #[inline]
    fn mul(self, velocity: Velocity) -> Velocity {
        Velocity(self * velocity.0)
    }
}

impl Mul<Acceleration> for SpatialTransform {
    type Output = Acceleration;
    #[inline]
    fn mul(self, acceleration: Acceleration) -> Acceleration {
        Acceleration(self * acceleration.0)
    }
}

impl Mul<ForceVector> for SpatialTransform {
    type Output = ForceVector;
    fn mul(self, force: ForceVector) -> ForceVector {
        let transform = self.0;
        let rotation_matrix = RotationMatrix::from(transform.rotation).get_value();
        let r_skew = transform.translation.vec().skew();

        let rotation =
            rotation_matrix * force.0.rotation - rotation_matrix * r_skew * force.0.translation;
        let translation = rotation_matrix * force.0.translation;
        ForceVector(SpatialVector::new(rotation, translation))
    }
}

impl Mul<Force> for SpatialTransform {
    type Output = Force;
    #[inline]
    fn mul(self, force: Force) -> Force {
        Force(self * force.0)
    }
}

impl Mul<Momentum> for SpatialTransform {
    type Output = Momentum;
    #[inline]
    fn mul(self, momentum: Momentum) -> Momentum {
        Momentum(self * momentum.0)
    }
}

impl Mul<SpatialTransform> for SpatialTransform {
    type Output = SpatialTransform;
    #[inline]
    fn mul(self, rhs: SpatialTransform) -> SpatialTransform {
        SpatialTransform::from(self.0 * rhs.0)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct SpatialInertia(pub MassProperties);

impl SpatialInertia {
    pub fn matrix(&self) -> Matrix6 {
        let mp = self.0;
        let inertia = mp.inertia;
        let mass = mp.mass;
        let com = mp.center_of_mass.vector();
        let cx = com.skew();
        let cxt = cx.transpose();

        let quad11 = inertia.matrix() + cx * cxt * mass;
        let quad12 = cx * mass;
        let quad21 = cxt * mass;
        let quad22 = Matrix3::identity() * mass;

        Matrix6::from_4matrix3(quad11, quad12, quad21, quad22)
    }
}

impl From<MassProperties> for SpatialInertia {
    fn from(value: MassProperties) -> SpatialInertia {
        SpatialInertia(value)
    }
}

impl Mul<SpatialVector> for SpatialInertia {
    type Output = SpatialVector;
    fn mul(self, v: SpatialVector) -> SpatialVector {
        SpatialVector::from(self.matrix() * v.vector())
    }
}

impl Mul<MotionVector> for SpatialInertia {
    type Output = ForceVector;
    fn mul(self, motion: MotionVector) -> ForceVector {
        ForceVector(self * motion.0)
    }
}

impl Mul<Velocity> for SpatialInertia {
    type Output = Momentum;
    fn mul(self, velocity: Velocity) -> Momentum {
        Momentum(self * velocity.0)
    }
}

impl Mul<Acceleration> for SpatialInertia {
    type Output = Force;
    fn mul(self, acceleration: Acceleration) -> Force {
        Force(self * acceleration.0)
    }
}
