use coordinate_systems::{
    cartesian::Cartesian, cylindrical::Cylindrical, spherical::Spherical, CoordinateSystem,
};

use rotations::{Rotation, RotationTrait};
use std::ops::Mul;

pub mod prelude {
    pub use coordinate_systems::prelude::*;
    pub use rotations::prelude::*;
}

/// ROTATION IS ACTUALLY A TRANSFORM(PASSIVE ROTATION), NOT AN ACTIVE ROTATION
#[derive(Debug, Default, Copy, Clone)]
pub struct Transform {
    pub rotation: Rotation,
    pub translation: CoordinateSystem,
}

impl Transform {
    pub fn new(rotation: Rotation, translation: CoordinateSystem) -> Self {
        Self {
            rotation,
            translation,
        }
    }
}

impl Mul<Transform> for Transform {
    type Output = Self;

    fn mul(self, rhs: Transform) -> Transform {
        // t1 is a transform from the f1 frame to the ref frame
        let t1 = &self;
        // t2 is a transform from the f2 frame to f1
        let t2 = &rhs;

        let translation_ref_to_t1_in_ref = t1.translation;
        let translation_t1_to_t2_in_t1 = t2.translation;

        // convert to cartesian so we can transform it, then convert back
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

#[cfg(test)]
mod tests {
    use super::*;
    use approx_eq::assert_approx_eq;
    use rotations::{
        euler_angles::{Angles, EulerAngles},
        quaternion::Quaternion,
    };
    use std::f64::consts::PI;
    const TOL: f64 = 1e-12;

    fn assert_transform_mul(
        angles1: Angles,
        translation1: Cartesian,
        angles2: Angles,
        translation2: Cartesian,
        expected_angles: Angles,
        expected_translation: Cartesian,
    ) {
        let rotation1 = EulerAngles::XYZ(angles1).into();
        let transform1 = Transform::new(rotation1, translation1.into());

        let rotation2 = EulerAngles::XYZ(angles2).into();
        let transform2 = Transform::new(rotation2, translation2.into());

        let result = transform1 * transform2;
        let result_rotation = Quaternion::from(result.rotation);
        let result_translation = Cartesian::from(result.translation);

        let expected_rotation = Quaternion::from(EulerAngles::XYZ(expected_angles));
        let expected_translation = expected_translation;

        assert_approx_eq!(result_rotation.s, expected_rotation.s, TOL);
        assert_approx_eq!(result_rotation.x, expected_rotation.x, TOL);
        assert_approx_eq!(result_rotation.y, expected_rotation.y, TOL);
        assert_approx_eq!(result_rotation.z, expected_rotation.z, TOL);
        assert_approx_eq!(result_translation.x, expected_translation.x, TOL);
        assert_approx_eq!(result_translation.y, expected_translation.y, TOL);
        assert_approx_eq!(result_translation.z, expected_translation.z, TOL);
    }

    #[test]
    fn test_transform_mul_xx() {
        let angles1 = Angles::new(PI / 2.0, 0.0, 0.0);
        let translation1 = Cartesian::new(1.0, 0.0, 0.0);
        let angles2 = Angles::new(PI / 2.0, 0.0, 0.0);
        let translation2 = Cartesian::new(1.0, 0.0, 0.0);
        let expected_angles = Angles::new(PI, 0.0, 0.0);
        let expected_translation = Cartesian::new(2.0, 0.0, 0.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
        );
    }

    #[test]
    fn test_transform_mul_yy() {
        let angles1 = Angles::new(0.0, PI / 2.0, 0.0);
        let translation1 = Cartesian::new(0.0, 1.0, 0.0);
        let angles2 = Angles::new(0.0, PI / 2.0, 0.0);
        let translation2 = Cartesian::new(0.0, 1.0, 0.0);
        let expected_angles = Angles::new(0.0, PI, 0.0);
        let expected_translation = Cartesian::new(0.0, 2.0, 0.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
        );
    }

    #[test]
    fn test_transform_mul_zz() {
        let angles1 = Angles::new(0.0, 0.0, PI / 2.0);
        let translation1 = Cartesian::new(0.0, 0.0, 1.0);
        let angles2 = Angles::new(0.0, 0.0, PI / 2.0);
        let translation2 = Cartesian::new(0.0, 0.0, 1.0);
        let expected_angles = Angles::new(0.0, 0.0, PI);
        let expected_translation = Cartesian::new(0.0, 0.0, 2.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
        );
    }

    #[test]
    fn test_transform_mul_xy() {
        let angles1 = Angles::new(PI / 2.0, 0.0, 0.0);
        let translation1 = Cartesian::new(1.0, 0.0, 0.0);
        let angles2 = Angles::new(0.0, PI / 2.0, 0.0);
        let translation2 = Cartesian::new(0.0, 1.0, 0.0);
        let expected_angles = Angles::new(PI / 2.0, PI / 2.0, 0.0);
        let expected_translation = Cartesian::new(1.0, 0.0, 1.0);

        assert_transform_mul(
            angles1,
            translation1,
            angles2,
            translation2,
            expected_angles,
            expected_translation,
        );
    }
}
