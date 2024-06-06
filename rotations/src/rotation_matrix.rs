use linear_algebra::{Matrix3, Vector3};
use std::ops::Mul;

#[derive(Debug, Copy, Clone)]
pub struct RotationMatrix {
    value: Matrix3,
}

#[derive(Debug, Copy, Clone)]
pub enum RotationMatrixError {
    ZeroMagnitudeColumn,
}

impl RotationMatrix {
    /// Creates a new `RotationMatrix` ensuring columns are normalized.
    ///
    /// # Arguments
    ///
    /// * `e11` - Element at row 1, column 1.
    /// * `e21` - Element at row 2, column 1.
    /// * `e31` - Element at row 3, column 1.
    /// * `e12` - Element at row 1, column 2.
    /// * `e22` - Element at row 2, column 2.
    /// * `e32` - Element at row 3, column 2.
    /// * `e13` - Element at row 1, column 3.
    /// * `e23` - Element at row 2, column 3.
    /// * `e33` - Element at row 3, column 3.
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

        let (e11, e21, e31) = normalize(e11, e21, e31)?;
        let (e12, e22, e32) = normalize(e12, e22, e32)?;
        let (e13, e23, e33) = normalize(e13, e23, e33)?;

        Ok(Self {
            value: Matrix3::new(e11, e21, e31, e12, e22, e32, e13, e23, e33),
        })
    }
}

impl Mul<Vector3> for RotationMatrix {
    type Output = Vector3;
    fn mul(self, rhs: Vector3) -> Vector3 {
        self.value * rhs
    }
}
