use super::*;
use linear_algebra::{Matrix3, Vector3};
use std::ops::Mul;

/// A struct representing a 3x3 rotation matrix.
#[derive(Debug, Copy, Clone)]
pub struct RotationMatrix {
    pub value: Matrix3,
}

/// Errors that can occur when creating a `RotationMatrix`.
#[derive(Debug, Copy, Clone)]
pub enum RotationMatrixError {
    /// Occurs when a column vector has zero magnitude.
    ZeroMagnitudeColumn,
}

impl RotationMatrix {
    /// Creates a `RotationMatrix` from a `Matrix3`.
    ///
    /// # Arguments
    ///
    /// * `value` - The 3x3 matrix representing the rotation.
    ///
    /// # Returns
    ///
    /// A new `RotationMatrix`.
    pub fn from_mat(value: Matrix3) -> Self {
        Self { value }
    }

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
        e21: f64,
        e31: f64,
        e12: f64,
        e22: f64,
        e32: f64,
        e13: f64,
        e23: f64,
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
        Ok(Self {
            value: Matrix3::new(e11, e21, e31, e12, e22, e32, e13, e23, e33),
        })
    }

    /// Creates an identity `RotationMatrix`.
    ///
    /// # Returns
    ///
    /// A new `RotationMatrix` representing the identity matrix.
    pub fn identity() -> Self {
        Self::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0).unwrap()
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
    fn rotate(&self, v: Vector3) -> Vector3 {
        self.value * v
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
    fn transform(&self, v: Vector3) -> Vector3 {
        self.value.transpose() * v
    }

    fn inv(&self) -> Self {
        let rotation_matrix = RotationMatrix::from(*self).value;
        RotationMatrix::from_mat(rotation_matrix.transpose())
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
        RotationMatrix::from_mat(self.value * rhs.value)
    }
}
