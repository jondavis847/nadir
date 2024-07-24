use coordinate_systems::{
    cartesian::Cartesian, cylindrical::Cylindrical, spherical::Spherical, CoordinateSystem,
};
use linear_algebra::vector3::Vector3;
use mass_properties::{CenterOfMass, Inertia, MassProperties};
use rotations::{rotation_matrix::RotationMatrix, Rotation, RotationTrait};
use std::ops::Mul;

pub mod prelude {  
    pub use crate::Transform;  
    pub use coordinate_systems::prelude::*;
    pub use rotations::prelude::*;
}

/// Represents the rotational and translational
/// Transform between two Frames
#[derive(Debug, Default, Copy, Clone)]
pub struct Transform {
    pub rotation: Rotation,
    pub translation: CoordinateSystem,
}

impl Transform {
    pub const IDENTITY: Self = Self {
        rotation: Rotation::IDENTITY,
        translation: CoordinateSystem::ZERO,
    };

    pub fn new(rotation: Rotation, translation: CoordinateSystem) -> Self {
        Self {
            rotation,
            translation,
        }
    }

    pub fn inv(&self) -> Self {
        let rotation = self.rotation.inv();
        let translation = -self.translation;
        Self {
            rotation,
            translation,
        }
    }
}

impl Mul<Vector3> for Transform {
    type Output = Vector3;
    /// T is a transform from A to B
    /// v is assumed to be values referenced in A
    /// the output should be v referenced in B
    fn mul(self, v_a: Vector3) -> Vector3 {
        // the translation of T is referenced in A, so just add the translation
        // translation is from A to B, so we need to take the negative since this is a transform, not motion
        let v_a_translated_to_b = v_a - Cartesian::from(self.translation).vec();
        // now transform via rotation
        let v_b = self.rotation.transform(v_a_translated_to_b);
        v_b
    }
}

impl Mul<Transform> for Transform {
    type Output = Self;

    /// The Transforms are implemented so that they work like Matrix multiplication.
    /// i.e. to take some vector V_A to V_C, V_C = T_B2C * T_B2A * V_A
    fn mul(self, rhs: Transform) -> Transform {
        // t1 is a transform from the ref frame to the f1 frame
        let t1 = &rhs;
        // t2 is a transform from the f1 frame to f2
        let t2 = &self;

        let translation_ref_to_t1_in_ref = t1.translation;
        let translation_t1_to_t2_in_t1 = t2.translation;

        // convert to cartesian (in case it's something else) so we can transform it, then convert back
        let cartesian_vec = Cartesian::from(translation_t1_to_t2_in_t1).vec();
        let rotated_vec = t1.rotation.inv().transform(cartesian_vec);
        let translation_t1_to_t2_in_ref = match translation_t1_to_t2_in_t1 {
            CoordinateSystem::Cartesian(_) => CoordinateSystem::from(Cartesian::from(rotated_vec)),
            CoordinateSystem::Cylindrical(_) => {
                CoordinateSystem::from(Cylindrical::from(rotated_vec))
            }
            CoordinateSystem::Spherical(_) => CoordinateSystem::from(Spherical::from(rotated_vec)),
        };

        let translation_ref_to_t2_in_ref =
            translation_ref_to_t1_in_ref + translation_t1_to_t2_in_ref;

        let new_rotation = self.rotation * rhs.rotation;
        let new_translation = translation_ref_to_t2_in_ref;

        Transform::new(new_rotation, new_translation)
    }
}

impl Mul<MassProperties> for Transform {
    type Output = MassProperties;
    fn mul(self, rhs: MassProperties) -> MassProperties {
        let mass = rhs.mass;

        let new_center_of_mass = CenterOfMass::from(self * rhs.center_of_mass.vector());

        let inertia = rhs.inertia.matrix();
        let rotation_matrix = RotationMatrix::from(self.rotation).get_value();

        // first transform the inertia (note passive rotation matrix from passive rotation)
        let rotated_inertia =
            Inertia::from(rotation_matrix.transpose() * inertia * rotation_matrix);

        // rotate the translation
        let translation = Cartesian::from(self.translation).vec();
        let rotated_translation = self.rotation.transform(translation);

        // apply parallel axis theorem
        let dx = rotated_translation.e1;
        let dy = rotated_translation.e2;
        let dz = rotated_translation.e3;

        let dx2 = dx * dx;
        let dy2 = dy * dy;
        let dz2 = dz * dz;

        let new_inertia = Inertia::new(
            rotated_inertia.get_ixx() + mass * (dy2 + dz2),
            rotated_inertia.get_ixy() - mass * dx * dy,
            rotated_inertia.get_ixz() - mass * dx * dz,
            rotated_inertia.get_iyy() + mass * (dx2 + dz2),
            rotated_inertia.get_iyz() - mass * dy * dz,
            rotated_inertia.get_izz() + mass * (dx2 + dy2),
        )
        .unwrap();

        MassProperties::new(mass, new_center_of_mass, new_inertia).unwrap()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx_eq::assert_approx_eq;
    use linear_algebra::vector3::Vector3;
    use rotations::{
        euler_angles::{EulerAngles, EulerSequence},
        quaternion::Quaternion,
    };
    use std::f64::consts::PI;
    const TOL: f64 = 1e-12;

    fn assert_transform_mul(
        angles1: EulerAngles,
        translation1: Cartesian,
        angles2: EulerAngles,
        translation2: Cartesian,
        expected_angles: EulerAngles,
        expected_translation: Cartesian,
        expected_vector: Vector3,
    ) {
        let rotation1 = angles1.into();
        let transform1 = Transform::new(rotation1, translation1.into());

        let rotation2 = angles2.into();
        let transform2 = Transform::new(rotation2, translation2.into());

        let result = transform2 * transform1;
        let result_rotation = Quaternion::from(result.rotation);
        let result_translation = Cartesian::from(result.translation);

        let expected_rotation = Quaternion::from(expected_angles);
        let expected_translation = expected_translation;

        let test_vector = Vector3::new(1.0, 2.0, 3.0);
        let result_vector = result * test_vector;

        assert_approx_eq!(result_rotation.s, expected_rotation.s, TOL);
        assert_approx_eq!(result_rotation.x, expected_rotation.x, TOL);
        assert_approx_eq!(result_rotation.y, expected_rotation.y, TOL);
        assert_approx_eq!(result_rotation.z, expected_rotation.z, TOL);
        assert_approx_eq!(result_translation.x, expected_translation.x, TOL);
        assert_approx_eq!(result_translation.y, expected_translation.y, TOL);
        assert_approx_eq!(result_translation.z, expected_translation.z, TOL);
        assert_approx_eq!(result_vector.e1, expected_vector.e1, TOL);
        assert_approx_eq!(result_vector.e2, expected_vector.e2, TOL);
        assert_approx_eq!(result_vector.e3, expected_vector.e3, TOL);
    }

    fn assert_transform_mul3(
        angles1: EulerAngles,
        translation1: Cartesian,
        angles2: EulerAngles,
        translation2: Cartesian,
        angles3: EulerAngles,
        translation3: Cartesian,
        expected_angles: EulerAngles,
        expected_translation: Cartesian,
        expected_vector: Vector3,
    ) {
        let rotation1 = angles1.into();
        let transform1 = Transform::new(rotation1, translation1.into());

        let rotation2 = angles2.into();
        let transform2 = Transform::new(rotation2, translation2.into());

        let rotation3 = angles3.into();
        let transform3 = Transform::new(rotation3, translation3.into());

        let result = transform3 * (transform2 * transform1);
        let result_rotation = Quaternion::from(result.rotation);
        let result_translation = Cartesian::from(result.translation);

        let expected_rotation = Quaternion::from(expected_angles);
        let expected_translation = expected_translation;

        let test_vector = Vector3::new(1.0, 2.0, 3.0);
        let result_vector = result * test_vector;

        assert_approx_eq!(result_rotation.s, expected_rotation.s, TOL);
        assert_approx_eq!(result_rotation.x, expected_rotation.x, TOL);
        assert_approx_eq!(result_rotation.y, expected_rotation.y, TOL);
        assert_approx_eq!(result_rotation.z, expected_rotation.z, TOL);
        assert_approx_eq!(result_translation.x, expected_translation.x, TOL);
        assert_approx_eq!(result_translation.y, expected_translation.y, TOL);
        assert_approx_eq!(result_translation.z, expected_translation.z, TOL);
        assert_approx_eq!(result_vector.e1, expected_vector.e1, TOL);
        assert_approx_eq!(result_vector.e2, expected_vector.e2, TOL);
        assert_approx_eq!(result_vector.e3, expected_vector.e3, TOL);
    }

    #[test]
    fn test_transform_mul_xx() {
        let angles1 = EulerAngles::new(PI / 2.0, 0.0, 0.0, EulerSequence::XYZ);
        let translation1 = Cartesian::new(1.0, 0.0, 0.0);
        let angles2 = EulerAngles::new(PI / 2.0, 0.0, 0.0, EulerSequence::XYZ);
        let translation2 = Cartesian::new(1.0, 0.0, 0.0);
        let expected_angles = EulerAngles::new(PI, 0.0, 0.0, EulerSequence::XYZ);
        let expected_translation = Cartesian::new(2.0, 0.0, 0.0);
        let expected_vector = Vector3::new(-1.0, -2.0, -3.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
            expected_vector,
        );
    }

    #[test]
    fn test_transform_mul_yy() {
        let angles1 = EulerAngles::new(0.0, PI / 2.0, 0.0, EulerSequence::XYZ);
        let translation1 = Cartesian::new(0.0, 1.0, 0.0);
        let angles2 = EulerAngles::new(0.0, PI / 2.0, 0.0, EulerSequence::XYZ);
        let translation2 = Cartesian::new(0.0, 1.0, 0.0);
        let expected_angles = EulerAngles::new(0.0, PI, 0.0, EulerSequence::XYZ);
        let expected_translation = Cartesian::new(0.0, 2.0, 0.0);
        let expected_vector = Vector3::new(-1.0, 0.0, -3.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
            expected_vector,
        );
    }

    #[test]
    fn test_transform_mul_zz() {
        let angles1 = EulerAngles::new(0.0, 0.0, PI / 2.0, EulerSequence::XYZ);
        let translation1 = Cartesian::new(0.0, 0.0, 1.0);
        let angles2 = EulerAngles::new(0.0, 0.0, PI / 2.0, EulerSequence::XYZ);
        let translation2 = Cartesian::new(0.0, 0.0, 1.0);
        let expected_angles = EulerAngles::new(0.0, 0.0, PI, EulerSequence::XYZ);
        let expected_translation = Cartesian::new(0.0, 0.0, 2.0);
        let expected_vector = Vector3::new(-1.0, -2.0, 1.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
            expected_vector,
        );
    }

    #[test]
    fn test_transform_mul_xy() {
        let angles1 =EulerAngles::new(PI / 2.0, 0.0, 0.0, EulerSequence::XYZ);
        let translation1 = Cartesian::new(1.0, 0.0, 0.0);
        let angles2 =EulerAngles::new(0.0, PI / 2.0, 0.0, EulerSequence::XYZ);
        let translation2 = Cartesian::new(0.0, 1.0, 0.0);
        let expected_angles =EulerAngles::new(PI / 2.0, PI / 2.0, 0.0, EulerSequence::XYZ);
        let expected_translation = Cartesian::new(1.0, 0.0, 1.0);
        let expected_vector = Vector3::new(2.0, 2.0, 0.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
            expected_vector,
        );
    }

    #[test]
    fn test_transform_mul_xz() {
        let angles1 =EulerAngles::new(PI / 2.0, 0.0, 0.0, EulerSequence::XYZ);
        let translation1 = Cartesian::new(1.0, 0.0, 0.0);
        let angles2 =EulerAngles::new(0.0, 0.0, PI / 2.0, EulerSequence::XYZ);
        let translation2 = Cartesian::new(0.0, 0.0, 1.0);
        let expected_angles =EulerAngles::new(PI / 2.0, 0.0, PI / 2.0, EulerSequence::XYZ);
        let expected_translation = Cartesian::new(1.0, -1.0, 0.0);
        let expected_vector = Vector3::new(3.0, 0.0, -3.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
            expected_vector,
        );
    }

    #[test]
    fn test_transform_mul_yz() {
        let angles1 =EulerAngles::new(0.0, PI / 2.0, 0.0, EulerSequence::XYZ);
        let translation1 = Cartesian::new(0.0, 1.0, 0.0);
        let angles2 =EulerAngles::new(0.0, 0.0, PI / 2.0, EulerSequence::XYZ);
        let translation2 = Cartesian::new(0.0, 0.0, 1.0);
        let expected_angles =EulerAngles::new(0.0, PI / 2.0, PI / 2.0, EulerSequence::XYZ);
        let expected_translation = Cartesian::new(1.0, 1.0, 0.0);
        let expected_vector = Vector3::new(1.0, 3.0, 0.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
            expected_vector,
        );
    }

    #[test]
    fn test_transform_mul_zyx() {
        let angles1 =EulerAngles::new(0.0, 0.0, PI / 2.0, EulerSequence::XYZ);
        let translation1 = Cartesian::new(0.0, 0.0, 1.0);
        let angles2 =EulerAngles::new(0.0, PI / 2.0, 0.0, EulerSequence::XYZ);
        let translation2 = Cartesian::new(0.0, 1.0, 0.0);
        let angles3 =EulerAngles::new(PI / 2.0, 0.0, 0.0, EulerSequence::XYZ);
        let translation3 = Cartesian::new(1.0, 0.0, 0.0);

        let expected_angles =EulerAngles::new(PI / 2.0, PI / 2.0, -PI / 2.0, EulerSequence::XYZ);
        let expected_translation = Cartesian::new(-1.0, 0.0, 0.0);
        let expected_vector = Vector3::new(-3.0, 2.0, 2.0);

        assert_transform_mul3(
            angles1,
            translation1,
            angles2,
            translation2,
            angles3,
            translation3,
            expected_angles,
            expected_translation,
            expected_vector,
        );
    }
}
