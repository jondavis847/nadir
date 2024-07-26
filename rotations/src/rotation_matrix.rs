use super::*;
use axes::{AlignedAxes, Axis};
use nalgebra::{Matrix3, Vector3};
use std::ops::Mul;

/// A struct representing a 3x3 rotation matrix.
#[derive(Debug, Copy, Clone, PartialEq)]
pub struct RotationMatrix(pub Matrix3<f64>);

/// Errors that can occur when creating a `RotationMatrix`.
#[derive(Debug, Copy, Clone)]
pub enum RotationMatrixError {
    /// Occurs when a column vector has zero magnitude.
    ZeroMagnitudeColumn,
}

impl RotationMatrix {
    /// Creates a new `RotationMatrix` ensuring columns are normalized.
    ///
    /// # Arguments
    ///
    /// * `e11`, `e21`, `e31` - Elements of the first column.
    /// * `e12`, `e22`, `e32` - Elements of the second column.
    /// * `e13`, `e23`, `e33` - Elements of the third column.
    ///
    /// # Returns
    ///
    /// A `Result` which is `Ok` containing a new `RotationMatrix` if columns are valid,
    /// or an `Err` containing a `RotationMatrixError`.
    pub fn new(
        e11: f64,
        e12: f64,
        e13: f64,
        e21: f64,
        e22: f64,
        e23: f64,
        e31: f64,
        e32: f64,
        e33: f64,
    ) -> Result<Self, RotationMatrixError> {
        // Helper function to normalize a column vector.
        fn normalize(e1: f64, e2: f64, e3: f64) -> Result<(f64, f64, f64), RotationMatrixError> {
            let mag_squared = e1 * e1 + e2 * e2 + e3 * e3;

            if mag_squared < f64::EPSILON {
                return Err(RotationMatrixError::ZeroMagnitudeColumn);
            }

            if (mag_squared - 1.0).abs() >= f64::EPSILON {
                let mag = mag_squared.sqrt();
                return Ok((e1 / mag, e2 / mag, e3 / mag));
            }

            Ok((e1, e2, e3))
        }

        // Normalize each column of the rotation matrix.
        let (e11, e21, e31) = normalize(e11, e21, e31)?;
        let (e12, e22, e32) = normalize(e12, e22, e32)?;
        let (e13, e23, e33) = normalize(e13, e23, e33)?;

        // Return the new `RotationMatrix` with normalized columns.
        Ok(Self(Matrix3::new(
            e11, e12, e13, e21, e22, e23, e31, e32, e33,
        )))
    }

    pub fn get_value(&self) -> Matrix3<f64> {
        self.0
    }
}

impl From<Matrix3<f64>> for RotationMatrix {
    fn from(value: Matrix3<f64>) -> Self {
        Self(value)
    }
}

impl From<Rotation> for RotationMatrix {
    fn from(rotation: Rotation) -> RotationMatrix {
        match rotation {
            Rotation::EulerAngles(rotation) => RotationMatrix::from(rotation),
            Rotation::Quaternion(rotation) => RotationMatrix::from(rotation),
            Rotation::RotationMatrix(rotation) => rotation,
        }
    }
}

impl From<Quaternion> for RotationMatrix {
    /// Converts a `Quaternion` into a `RotationMatrix`.
    ///
    /// # Arguments
    ///
    /// * `q` - The quaternion to be converted.
    ///
    /// # Returns
    ///
    /// A new `RotationMatrix` representing the rotation defined by the quaternion.
    fn from(q: Quaternion) -> Self {
        let s = q.s;
        let x = q.x;
        let y = q.y;
        let z = q.z;

        let e11 = 1.0 - 2.0 * y * y - 2.0 * z * z;
        let e12 = 2.0 * x * y + 2.0 * s * z;
        let e13 = 2.0 * x * z - 2.0 * s * y;
        let e21 = 2.0 * x * y - 2.0 * s * z;
        let e22 = 1.0 - 2.0 * x * x - 2.0 * z * z;
        let e23 = 2.0 * y * z - 2.0 * s * x;
        let e31 = 2.0 * x * z + 2.0 * s * y;
        let e32 = 2.0 * y * z - 2.0 * s * x;
        let e33 = 1.0 - 2.0 * x * x - 2.0 * y * y;

        // Create the `RotationMatrix` from the computed elements.
        RotationMatrix::new(e11, e21, e31, e12, e22, e32, e13, e23, e33).unwrap()
    }
}

impl From<EulerAngles> for RotationMatrix {
    /// Converts `EulerAngles` into a `RotationMatrix`.
    ///
    /// # Arguments
    ///
    /// * `euler_angles` - The euler_angles to be converted.
    ///
    /// # Returns
    ///
    /// A new `RotationMatrix` representing the rotation defined by the euler angles.
    fn from(euler_angles: EulerAngles) -> RotationMatrix {
        let rotx =
            |a: f64| Matrix3::new(1.0, 0.0, 0.0, 0.0, a.cos(), -a.sin(), 0.0, a.sin(), a.cos());
        let roty =
            |a: f64| Matrix3::new(a.cos(), 0.0, a.sin(), 0.0, 1.0, 0.0, -a.sin(), 0.0, a.cos());
        let rotz =
            |a: f64| Matrix3::new(a.cos(), -a.sin(), 0.0, a.sin(), a.cos(), 0.0, 0.0, 0.0, 1.0);

        let (phi, theta, psi) = (euler_angles.phi, euler_angles.theta, euler_angles.psi);
        match euler_angles.sequence {
            EulerSequence::XYZ => RotationMatrix::from(rotz(psi) * roty(theta) * rotx(phi)),
            EulerSequence::XZY => RotationMatrix::from(roty(psi) * rotz(theta) * rotx(phi)),
            EulerSequence::YXZ => RotationMatrix::from(rotz(psi) * rotx(theta) * roty(phi)),
            EulerSequence::YZX => RotationMatrix::from(rotx(psi) * rotz(theta) * roty(phi)),
            EulerSequence::ZXY => RotationMatrix::from(roty(psi) * rotx(theta) * rotz(phi)),
            EulerSequence::ZYX => RotationMatrix::from(rotx(psi) * roty(theta) * rotz(phi)),
            EulerSequence::XYX => RotationMatrix::from(rotx(psi) * roty(theta) * rotx(phi)),
            EulerSequence::YXY => RotationMatrix::from(roty(psi) * rotx(theta) * roty(phi)),
            EulerSequence::XZX => RotationMatrix::from(rotx(psi) * rotz(theta) * rotx(phi)),
            EulerSequence::ZXZ => RotationMatrix::from(rotz(psi) * rotx(theta) * rotz(phi)),
            EulerSequence::YZY => RotationMatrix::from(roty(psi) * rotz(theta) * roty(phi)),
            EulerSequence::ZYZ => RotationMatrix::from(rotz(psi) * roty(theta) * rotz(phi)),
        }
    }
}

impl From<AlignedAxes> for RotationMatrix {
    fn from(axes: AlignedAxes) -> Self {
        let mut m = Matrix3::zeros();

        let axis_to_vector = |axis: Axis| -> Vector3<f64> {
            match axis {
                Axis::Xp => Vector3::new(1.0, 0.0, 0.0),
                Axis::Xn => Vector3::new(-1.0, 0.0, 0.0),
                Axis::Yp => Vector3::new(0.0, 1.0, 0.0),
                Axis::Yn => Vector3::new(0.0, -1.0, 0.0),
                Axis::Zp => Vector3::new(0.0, 0.0, 1.0),
                Axis::Zn => Vector3::new(0.0, 0.0, -1.0),
            }
        };

        let primary_new_vec = axis_to_vector(axes.primary.new);
        let secondary_new_vec = axis_to_vector(axes.secondary.new);

        let set_column = |m: &mut Matrix3<f64>, col_idx: usize, vec: Vector3<f64>| {
            m[(0, col_idx)] = vec[0];
            m[(1, col_idx)] = vec[1];
            m[(2, col_idx)] = vec[2];
        };

        // Determine columns based on old primary and secondary axes
        let primary_col = match axes.primary.old {
            Axis::Xp | Axis::Xn => 0,
            Axis::Yp | Axis::Yn => 1,
            Axis::Zp | Axis::Zn => 2,
        };

        let secondary_col = match axes.secondary.old {
            Axis::Xp | Axis::Xn => 0,
            Axis::Yp | Axis::Yn => 1,
            Axis::Zp | Axis::Zn => 2,
        };

        // Place the new vectors in the correct columns
        set_column(&mut m, primary_col, primary_new_vec);
        set_column(&mut m, secondary_col, secondary_new_vec);

        let primary_vec = Vector3::new(
            m[(0, primary_col)],
            m[(1, primary_col)],
            m[(2, primary_col)],
        );
        let secondary_vec = Vector3::new(
            m[(0, secondary_col)],
            m[(1, secondary_col)],
            m[(2, secondary_col)],
        );

        let (third_col, third_vec) = match (primary_col, secondary_col) {
            (0, 1) => (2, primary_vec.cross(&secondary_vec)),
            (0, 2) => (1, secondary_vec.cross(&primary_vec)),
            (1, 0) => (2, secondary_vec.cross(&primary_vec)),
            (1, 2) => (0, primary_vec.cross(&secondary_vec)),
            (2, 0) => (1, primary_vec.cross(&secondary_vec)),
            (2, 1) => (0, secondary_vec.cross(&primary_vec)),
            _ => panic!("invalid combo"),
        };
        set_column(&mut m, third_col, third_vec);

        RotationMatrix::from(m)
    }
}

impl RotationTrait for RotationMatrix {
    /// Rotates a vector by the rotation matrix.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be rotated.
    ///
    /// # Returns
    ///
    /// The rotated vector.
    fn rotate(&self, v: Vector3<f64>) -> Vector3<f64> {
        self.0 * v
    }

    /// Transforms a vector by the transpose of the rotation matrix.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to be transformed.
    ///
    /// # Returns
    ///
    /// The transformed vector.
    fn transform(&self, v: Vector3<f64>) -> Vector3<f64> {
        self.0.transpose() * v
    }

    fn inv(&self) -> Self {
        RotationMatrix::from(self.0.transpose())
    }

    /// Creates an identity `RotationMatrix`.
    ///
    /// # Returns
    ///
    /// A new `RotationMatrix` representing the identity matrix.
    fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0).unwrap()
    }
}

impl Mul<RotationMatrix> for RotationMatrix {
    type Output = RotationMatrix;

    /// Multiplies two rotation matrices.
    ///
    /// # Arguments
    ///
    /// * `rhs` - The right-hand side rotation matrix.
    ///
    /// # Returns
    ///
    /// A new `RotationMatrix` representing the product of the two rotation matrices.
    fn mul(self, rhs: RotationMatrix) -> RotationMatrix {
        RotationMatrix::from(self.0 * rhs.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use axes::AxisPair;
    use nalgebra::Matrix3;

    #[test]
    fn test_rotation_from_aligned_axes_1() {
        let axis_rotation = AlignedAxes::new(
            AxisPair {
                old: Axis::Xp,
                new: Axis::Xn,
            },
            AxisPair {
                old: Axis::Yp,
                new: Axis::Yp,
            },
        );
        let rotation = RotationMatrix::from(axis_rotation);
        let expected_matrix = Matrix3::new(-1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0);
        assert_eq!(rotation.0, expected_matrix);
    }

    #[test]
    fn test_rotation_from_aligned_axes_2() {
        let axis_rotation = AlignedAxes::new(
            AxisPair {
                old: Axis::Yp,
                new: Axis::Yn,
            },
            AxisPair {
                old: Axis::Zp,
                new: Axis::Zp,
            },
        );
        let rotation = RotationMatrix::from(axis_rotation);
        let expected_matrix = Matrix3::new(-1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 1.0);
        assert_eq!(rotation.0, expected_matrix);
    }

    #[test]
    fn test_rotation_from_aligned_axes_3() {
        let axis_rotation = AlignedAxes::new(
            AxisPair {
                old: Axis::Zp,
                new: Axis::Zn,
            },
            AxisPair {
                old: Axis::Xp,
                new: Axis::Xp,
            },
        );
        let rotation = RotationMatrix::from(axis_rotation);
        let expected_matrix = Matrix3::new(1.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, -1.0);

        assert_eq!(rotation.0, expected_matrix);
    }

    #[test]
    fn test_rotation_from_aligned_axes_4() {
        let axis_rotation = AlignedAxes::new(
            AxisPair {
                old: Axis::Xp,
                new: Axis::Yp,
            },
            AxisPair {
                old: Axis::Zp,
                new: Axis::Xp,
            },
        );
        let rotation = RotationMatrix::from(axis_rotation);        
        let expected_matrix = Matrix3::new(0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert_eq!(rotation.0, expected_matrix);
    }

    #[test]
    fn test_rotation_from_aligned_axes_5() {
        let axis_rotation = AlignedAxes::new(
            AxisPair {
                old: Axis::Yn,
                new: Axis::Zn,
            },
            AxisPair {
                old: Axis::Xp,
                new: Axis::Yn,
            },
        );
        let rotation = RotationMatrix::from(axis_rotation);
        let expected_matrix = Matrix3::new(0.0, 0.0, -1.0, -1.0, 0.0, 0.0, 0.0, 1.0, 0.0);
        assert_eq!(rotation.0, expected_matrix);
    }

    #[test]
    fn test_rotation_from_aligned_axes_6() {
        let axis_rotation = AlignedAxes::new(
            AxisPair {
                old: Axis::Zn,
                new: Axis::Yn,
            },
            AxisPair {
                old: Axis::Xp,
                new: Axis::Xp,
            },
        );
        let rotation = RotationMatrix::from(axis_rotation);
        let expected_matrix = Matrix3::new(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, -1.0, 0.0);
        assert_eq!(rotation.0, expected_matrix);
    }

    #[test]
    fn test_rotation_from_aligned_axes_7() {
        let axis_rotation = AlignedAxes::new(
            AxisPair {
                old: Axis::Xn,
                new: Axis::Zp,
            },
            AxisPair {
                old: Axis::Yn,
                new: Axis::Xp,
            },
        );
        let rotation = RotationMatrix::from(axis_rotation);
        let expected_matrix = Matrix3::new(0.0, -1.0, 0.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0);
        assert_eq!(rotation.0, expected_matrix);
    }

    #[test]
    fn test_rotation_from_aligned_axes_8() {
        let axis_rotation = AlignedAxes::new(
            AxisPair {
                old: Axis::Yp,
                new: Axis::Xn,
            },
            AxisPair {
                old: Axis::Zp,
                new: Axis::Yn,
            },
        );
        let rotation = RotationMatrix::from(axis_rotation);
        let expected_matrix = Matrix3::new(0.0, -1.0, -0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0);
        assert_eq!(rotation.0, expected_matrix);
    }
}
