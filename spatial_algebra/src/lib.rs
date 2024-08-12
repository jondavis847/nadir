use mass_properties::{CenterOfMass, Inertia, MassProperties};
use nalgebra::{Matrix3, Matrix6, Vector3, Vector6};
use rotations::rotation_matrix::RotationMatrix;
use serde::{Serialize, Deserialize};
use std::ops::{Add, Mul, Sub};
use transforms::Transform;

#[derive(Clone, Copy, Debug, Default)]
pub struct SpatialVector {
    pub rotation: Vector3<f64>,    //TODO should we force this as Cartesian?
    pub translation: Vector3<f64>, //TODO should we force this as Cartesian?
}

impl SpatialVector {
    pub fn new(rotation: Vector3<f64>, translation: Vector3<f64>) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub fn get_index(&self, index: usize) -> Option<f64> {
        match index {
            1 => Some(self.rotation[0]),
            2 => Some(self.rotation[1]),
            3 => Some(self.rotation[2]),
            4 => Some(self.translation[0]),
            5 => Some(self.translation[1]),
            6 => Some(self.translation[2]),
            _ => None,
        }
    }

    #[inline]
    pub fn vector(&self) -> Vector6<f64> {
        Vector6::new(
            self.rotation[0],
            self.rotation[1],
            self.rotation[2],
            self.translation[0],
            self.translation[1],
            self.translation[2],
        )
    }
    ///Featherstone 2.34
    pub fn cross_force(self, rhs: SpatialVector) -> SpatialVector {
        let lhs_rotation = self.rotation;
        let lhs_translation = self.translation;
        let rhs_rotation = rhs.rotation;
        let rhs_translation = rhs.translation;

        let new_rotation =
            lhs_rotation.cross(&rhs_rotation) + lhs_translation.cross(&rhs_translation);
        let new_translation = lhs_rotation.cross(&rhs_translation);
        SpatialVector::new(new_rotation, new_translation)
    }
    /// Featherstone 2.33
    pub fn cross_motion(self, rhs: SpatialVector) -> SpatialVector {
        let lhs_rotation = self.rotation;
        let lhs_translation = self.translation;
        let rhs_rotation = rhs.rotation;
        let rhs_translation = rhs.translation;

        let new_rotation = lhs_rotation.cross(&rhs_rotation);
        let new_translation =
            lhs_rotation.cross(&rhs_translation) + lhs_translation.cross(&rhs_rotation);
        SpatialVector::new(new_rotation, new_translation)
    }
}

impl From<Vector6<f64>> for SpatialVector {
    #[inline]
    fn from(v: Vector6<f64>) -> SpatialVector {
        let rotation = Vector3::new(v[0], v[1], v[2]);
        let translation = Vector3::new(v[3], v[4], v[5]);
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
pub struct MotionVector(pub SpatialVector);

impl MotionVector {
    #[inline]
    pub fn vector(&self) -> Vector6<f64> {
        self.0.vector()
    }

    #[inline]
    pub fn cross_motion(self, rhs: MotionVector) -> MotionVector {
        MotionVector(self.0.cross_motion(rhs.0))
    }

    #[inline]
    pub fn cross_force(self, rhs: ForceVector) -> ForceVector {
        ForceVector(self.0.cross_force(rhs.0))
    }

    #[inline]
    pub fn get_index(&self, index: usize) -> Option<f64> {
        self.0.get_index(index)
    }

    #[inline]
    pub fn rotation(&self) -> &Vector3<f64> {
        &self.0.rotation
    }

    #[inline]
    pub fn translation(&self) -> &Vector3<f64> {
        &self.0.translation
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
pub struct Velocity(pub MotionVector);

impl Velocity {
    #[inline]
    pub fn vector(&self) -> Vector6<f64> {
        self.0.vector()
    }

    #[inline]
    pub fn cross_motion(self, rhs: Velocity) -> Acceleration {
        Acceleration(self.0.cross_motion(rhs.0))
    }

    #[inline]
    pub fn cross_force(self, rhs: Momentum) -> Force {
        Force(self.0.cross_force(rhs.0))
    }

    #[inline]
    pub fn get_index(&self, index: usize) -> Option<f64> {
        self.0.get_index(index)
    }

    #[inline]
    pub fn rotation(&self) -> &Vector3<f64> {
        &self.0.rotation()
    }

    #[inline]
    pub fn translation(&self) -> &Vector3<f64> {
        &self.0.translation()
    }

    #[inline]
    pub fn zeros() -> Self {
        Self(MotionVector(SpatialVector::new(
            Vector3::zeros(),
            Vector3::zeros(),
        )))
    }
}

impl From<Vector6<f64>> for Velocity {
    #[inline]
    fn from(v: Vector6<f64>) -> Self {
        Velocity(MotionVector(SpatialVector::from(v)))
    }
}

impl From<MotionVector> for Velocity {
    #[inline]
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
pub struct Acceleration(pub MotionVector);

impl Acceleration {
    #[inline]
    pub fn vector(&self) -> Vector6<f64> {
        self.0.vector()
    }

    #[inline]
    pub fn zeros() -> Self {
        Self(MotionVector(SpatialVector::new(
            Vector3::zeros(),
            Vector3::zeros(),
        )))
    }

    #[inline]
    pub fn rotation(&self) -> &Vector3<f64> {
        &self.0.rotation()
    }

    #[inline]
    pub fn translation(&self) -> &Vector3<f64> {
        &self.0.translation()
    }
}

impl Add<Acceleration> for Acceleration {
    type Output = Acceleration;
    #[inline]
    fn add(self, rhs: Acceleration) -> Acceleration {
        Acceleration(self.0 + rhs.0)
    }
}

impl From<Vector6<f64>> for Acceleration {
    #[inline]
    fn from(v: Vector6<f64>) -> Self {
        Acceleration(MotionVector(SpatialVector::from(v)))
    }
}

impl From<MotionVector> for Acceleration {
    #[inline]
    fn from(motion: MotionVector) -> Self {
        Self(motion)
    }
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ForceVector(SpatialVector);

impl ForceVector {
    #[inline]
    pub fn get_index(&self, index: usize) -> Option<f64> {
        self.0.get_index(index)
    }

    #[inline]
    pub fn vector(&self) -> Vector6<f64> {
        self.0.vector()
    }

    #[inline]
    pub fn rotation(&self) -> &Vector3<f64> {
        &self.0.rotation
    }

    #[inline]
    pub fn translation(&self) -> &Vector3<f64> {
        &self.0.translation
    }
}

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

impl Momentum {
    #[inline]
    pub fn vector(&self) -> Vector6<f64> {
        self.0.vector()
    }

    #[inline]
    pub fn rotation(&self) -> &Vector3<f64> {
        &self.0.rotation()
    }

    #[inline]
    pub fn translation(&self) -> &Vector3<f64> {
        &self.0.translation()
    }
}

impl From<Vector6<f64>> for Momentum {
    #[inline]
    fn from(v: Vector6<f64>) -> Self {
        Momentum(ForceVector(SpatialVector::from(v)))
    }
}

impl From<ForceVector> for Momentum {
    #[inline]
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

impl Force {
    #[inline]
    pub fn get_index(&self, index: usize) -> Option<f64> {
        self.0.get_index(index)
    }

    #[inline]
    pub fn vector(&self) -> Vector6<f64> {
        self.0.vector()
    }

    #[inline]
    pub fn rotation(&self) -> &Vector3<f64> {
        &self.0.rotation()
    }

    #[inline]
    pub fn translation(&self) -> &Vector3<f64> {
        &self.0.translation()
    }
}

impl From<Vector6<f64>> for Force {
    #[inline]
    fn from(v: Vector6<f64>) -> Self {
        Force(ForceVector(SpatialVector::from(v)))
    }
}

impl From<ForceVector> for Force {
    #[inline]
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
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct SpatialTransform(pub Transform);

impl SpatialTransform {
    #[inline]
    pub fn inv(&self) -> SpatialTransform {
        SpatialTransform::from(self.0.inv())
    }

    pub fn matrix_motion(&self) -> Matrix6<f64> {
        let transform = self.0;
        let rotation_matrix = RotationMatrix::from(transform.rotation).get_value();
        let r_skew = transform.translation.vec().cross_matrix();

        let m11 = rotation_matrix;
        let m12 = Matrix3::zeros();
        let m21 = -rotation_matrix * r_skew;
        let m22 = rotation_matrix;

        let mut m6 = Matrix6::zeros();

        m6.index_mut((0..3, 0..3)).copy_from(&m11);
        m6.index_mut((0..3, 3..6)).copy_from(&m12);
        m6.index_mut((3..6, 0..3)).copy_from(&m21);
        m6.index_mut((3..6, 3..6)).copy_from(&m22);
        m6
    }

    pub fn matrix_force(&self) -> Matrix6<f64> {
        let transform = self.0;
        let rotation_matrix = RotationMatrix::from(transform.rotation).get_value();
        let r_skew = transform.translation.vec().cross_matrix();

        let m11 = rotation_matrix;
        let m12 = -rotation_matrix * r_skew;
        let m21 = Matrix3::zeros();
        let m22 = rotation_matrix;

        let mut m6 = Matrix6::zeros();

        m6.index_mut((0..3, 0..3)).copy_from(&m11);
        m6.index_mut((0..3, 3..6)).copy_from(&m12);
        m6.index_mut((3..6, 0..3)).copy_from(&m21);
        m6.index_mut((3..6, 3..6)).copy_from(&m22);
        m6
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
        let r_skew = transform.translation.vec().cross_matrix();        
        
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
        let r_skew = transform.translation.vec().cross_matrix();

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

impl Mul<SpatialInertia> for SpatialTransform {
    type Output = SpatialInertia;
    fn mul(self, inertia: SpatialInertia) -> SpatialInertia {
        let transform_motion = self.matrix_motion();
        let transform_force = self.matrix_force();        
        SpatialInertia::from(transform_force * inertia.matrix() * transform_motion.try_inverse().unwrap())
    }
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct SpatialInertia(pub Matrix6<f64>);

impl SpatialInertia {
    pub fn matrix(&self) -> Matrix6<f64> {
        self.0
    }

    pub fn to_mass_properties(&self) -> MassProperties {
        let m = self.0;
        let mass = m[(5, 5)];
        let mc = m.fixed_view::<3, 3>(0, 3);

        let mc_unskew = Vector3::new(
            mc[(2, 1)] - mc[(1, 2)],
            mc[(0, 2)] - mc[(2, 0)],
            mc[(1, 0)] - mc[(0, 1)],
        ) * 0.5;

        let center_of_mass = CenterOfMass::from(mc_unskew / mass);
        let inertia = Inertia::from(m.fixed_view::<3, 3>(0, 0) - mc * mc.transpose() / mass);
        MassProperties::new(mass, center_of_mass, inertia).unwrap()
    }
}

impl From<Matrix6<f64>> for SpatialInertia {
    fn from(m: Matrix6<f64>) -> Self {
        SpatialInertia(m)
    }
}

impl From<MassProperties> for SpatialInertia {
    fn from(mp: MassProperties) -> Self {
        let inertia = mp.inertia;
        let mass = mp.mass;
        let com = mp.center_of_mass.vector();

        let cx = com.cross_matrix();
        let cxt = cx.transpose();

        let m11 = inertia.matrix() + cx * cxt * mass;
        let m12 = cx * mass;
        let m21 = cxt * mass;
        let m22 = Matrix3::identity() * mass;

        let mut m6 = Matrix6::zeros();

        m6.index_mut((0..3, 0..3)).copy_from(&m11);
        m6.index_mut((0..3, 3..6)).copy_from(&m12);
        m6.index_mut((3..6, 0..3)).copy_from(&m21);
        m6.index_mut((3..6, 3..6)).copy_from(&m22);
        m6.into()
    }
}

impl Add<SpatialInertia> for SpatialInertia {
    type Output = SpatialInertia;
    fn add(self, rhs: SpatialInertia) -> SpatialInertia {
        SpatialInertia::from(self.matrix() + rhs.matrix())
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

//TODO: MORE TESTS!
#[cfg(test)]
mod tests {
    use coordinate_systems::cartesian::Cartesian;
    use rotations::{prelude::EulerAngles, Rotation};

    use super::*;
    const TOL: f64 = 1e-12;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected,
            actual
        );
    }
    
    #[test]
    fn test_transform_mul_acceleration() {
        let accel = Acceleration::from(Vector6::new(-1.0, 0.0, 0.0, 0.0, 0.0, 0.0));
        let translation = Cartesian::new(0.0, 0.0, -1.0);
        let transform = Transform::new(Rotation::IDENTITY, translation.into());
        let spatial_transform = SpatialTransform::from(transform);
        let result = spatial_transform * accel;

        assert_close(result.vector()[0], -1.0);
        assert_close(result.vector()[1], 0.0);
        assert_close(result.vector()[2], 0.0);
        assert_close(result.vector()[3], 0.0);
        assert_close(result.vector()[4], -1.0);
        assert_close(result.vector()[5], 0.0);
    }


    #[test]
    fn test_transform_inertia_translation() {
        let inertia = SpatialInertia::from(MassProperties::default());        
        let translation = Cartesian::new(0.0, 0.0, 1.0);
        let transform = Transform::new(Rotation::IDENTITY, translation.into());
        let spatial_transform = SpatialTransform::from(transform);
        let result = spatial_transform * inertia;

        assert_close(result.0[(0,0)], 2.0);
        assert_close(result.0[(1,0)], 0.0);
        assert_close(result.0[(2,0)], 0.0);
        assert_close(result.0[(3,0)], 0.0);
        assert_close(result.0[(4,0)], 1.0);
        assert_close(result.0[(5,0)], 0.0);
        assert_close(result.0[(0,1)], 0.0);
        assert_close(result.0[(1,1)], 2.0);
        assert_close(result.0[(2,1)], 0.0);
        assert_close(result.0[(3,1)], -1.0);
        assert_close(result.0[(4,1)], 0.0);
        assert_close(result.0[(5,1)], 0.0);
        assert_close(result.0[(0,2)], 0.0);
        assert_close(result.0[(1,2)], 0.0);
        assert_close(result.0[(2,2)], 1.0);
        assert_close(result.0[(3,2)], 0.0);
        assert_close(result.0[(4,2)], 0.0);
        assert_close(result.0[(5,2)], 0.0);
        assert_close(result.0[(0,3)], 0.0);
        assert_close(result.0[(1,3)], -1.0);
        assert_close(result.0[(2,3)], 0.0);
        assert_close(result.0[(3,3)], 1.0);
        assert_close(result.0[(4,3)], 0.0);
        assert_close(result.0[(5,3)], 0.0);
        assert_close(result.0[(0,4)], 1.0);
        assert_close(result.0[(1,4)], 0.0);
        assert_close(result.0[(2,4)], 0.0);
        assert_close(result.0[(3,4)], 0.0);
        assert_close(result.0[(4,4)], 1.0);
        assert_close(result.0[(5,4)], 0.0);
        assert_close(result.0[(0,5)], 0.0);
        assert_close(result.0[(1,5)], 0.0);
        assert_close(result.0[(2,5)], 0.0);
        assert_close(result.0[(3,5)], 0.0);
        assert_close(result.0[(4,5)], 0.0);
        assert_close(result.0[(5,5)], 1.0);
    }

    #[test]
    fn test_transform_inertia_translation_2() {
        let inertia = SpatialInertia::from(MassProperties::default());        
        let translation1 = Cartesian::new(0.0, 0.0, 0.5);
        let transform1 = Transform::new(Rotation::IDENTITY, translation1.into());
        let spatial_transform1 = SpatialTransform::from(transform1);
        let translation2 = Cartesian::new(0.0, 0.0, 1.0);
        let transform2 = Transform::new(Rotation::IDENTITY, translation2.into());
        let spatial_transform2 = SpatialTransform::from(transform2);

        let inertia1 = spatial_transform1 * inertia;

        let result = inertia1 + spatial_transform2 * inertia1;

        assert_close(result.0[(0,0)], 4.5);
        assert_close(result.0[(1,0)], 0.0);
        assert_close(result.0[(2,0)], 0.0);
        assert_close(result.0[(3,0)], 0.0);
        assert_close(result.0[(4,0)], 2.0);
        assert_close(result.0[(5,0)], 0.0);
        assert_close(result.0[(0,1)], 0.0);
        assert_close(result.0[(1,1)], 4.5);
        assert_close(result.0[(2,1)], 0.0);
        assert_close(result.0[(3,1)], -2.0);
        assert_close(result.0[(4,1)], 0.0);
        assert_close(result.0[(5,1)], 0.0);
        assert_close(result.0[(0,2)], 0.0);
        assert_close(result.0[(1,2)], 0.0);
        assert_close(result.0[(2,2)], 2.0);
        assert_close(result.0[(3,2)], 0.0);
        assert_close(result.0[(4,2)], 0.0);
        assert_close(result.0[(5,2)], 0.0);
        assert_close(result.0[(0,3)], 0.0);
        assert_close(result.0[(1,3)], -2.0);
        assert_close(result.0[(2,3)], 0.0);
        assert_close(result.0[(3,3)], 2.0);
        assert_close(result.0[(4,3)], 0.0);
        assert_close(result.0[(5,3)], 0.0);
        assert_close(result.0[(0,4)], 2.0);
        assert_close(result.0[(1,4)], 0.0);
        assert_close(result.0[(2,4)], 0.0);
        assert_close(result.0[(3,4)], 0.0);
        assert_close(result.0[(4,4)], 2.0);
        assert_close(result.0[(5,4)], 0.0);
        assert_close(result.0[(0,5)], 0.0);
        assert_close(result.0[(1,5)], 0.0);
        assert_close(result.0[(2,5)], 0.0);
        assert_close(result.0[(3,5)], 0.0);
        assert_close(result.0[(4,5)], 0.0);
        assert_close(result.0[(5,5)], 2.0);
    }

    #[test]
    fn test_transform_inertia_translation_and_rotation() {
        let inertia = SpatialInertia::from(MassProperties::default());        
        let translation = Cartesian::new(0.0, 0.0, 1.0);
        let rotation = EulerAngles::new(std::f64::consts::FRAC_PI_2,0.0,0.0, rotations::prelude::EulerSequence::XYZ);
        let transform = Transform::new(rotation.into(), translation.into());
        let spatial_transform = SpatialTransform::from(transform);
        let result = spatial_transform * inertia;

        assert_close(result.0[(0,0)], 2.0);
        assert_close(result.0[(1,0)], 0.0);
        assert_close(result.0[(2,0)], 0.0);
        assert_close(result.0[(3,0)], 0.0);
        assert_close(result.0[(4,0)], 0.0);
        assert_close(result.0[(5,0)], -1.0);
        assert_close(result.0[(0,1)], 0.0);
        assert_close(result.0[(1,1)], 1.0);
        assert_close(result.0[(2,1)], 0.0);
        assert_close(result.0[(3,1)], 0.0);
        assert_close(result.0[(4,1)], 0.0);
        assert_close(result.0[(5,1)], 0.0);
        assert_close(result.0[(0,2)], 0.0);
        assert_close(result.0[(1,2)], 0.0);
        assert_close(result.0[(2,2)], 2.0);
        assert_close(result.0[(3,2)], 1.0);
        assert_close(result.0[(4,2)], 0.0);
        assert_close(result.0[(5,2)], 0.0);
        assert_close(result.0[(0,3)], 0.0);
        assert_close(result.0[(1,3)], 0.0);
        assert_close(result.0[(2,3)], 1.0);
        assert_close(result.0[(3,3)], 1.0);
        assert_close(result.0[(4,3)], 0.0);
        assert_close(result.0[(5,3)], 0.0);
        assert_close(result.0[(0,4)], 0.0);
        assert_close(result.0[(1,4)], 0.0);
        assert_close(result.0[(2,4)], 0.0);
        assert_close(result.0[(3,4)], 0.0);
        assert_close(result.0[(4,4)], 1.0);
        assert_close(result.0[(5,4)], 0.0);
        assert_close(result.0[(0,5)], -1.0);
        assert_close(result.0[(1,5)], 0.0);
        assert_close(result.0[(2,5)], 0.0);
        assert_close(result.0[(3,5)], 0.0);
        assert_close(result.0[(4,5)], 0.0);
        assert_close(result.0[(5,5)], 1.0);
    }

}