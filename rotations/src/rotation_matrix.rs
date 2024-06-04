use sim_value::SimValue;
use linear_algebra::{Matrix3, Vector3};
use std::ops::Mul;

#[derive(Debug, Copy, Clone)]
pub struct RotationMatrix<T>
where
    T: SimValue,
{
    value: Matrix3<T>,
}

#[derive(Debug, Copy, Clone)]
pub enum RotationMatrixError {
    ZeroMagnitudeColumn,
}

impl<T> RotationMatrix<T>
where
    T: SimValue,
{
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
        e11: T,
        e21: T,
        e31: T,
        e12: T,
        e22: T,
        e32: T,
        e13: T,
        e23: T,
        e33: T,
    ) -> Result<Self, RotationMatrixError> {
        fn normalize<T>(e1: T, e2: T, e3: T) -> Result<(T, T, T), RotationMatrixError>
        where
            T: SimValue,
        {
            let mag_squared = e1 * e1 + e2 * e2 + e3 * e3;

            if mag_squared < T::EPSILON {
                return Err(RotationMatrixError::ZeroMagnitudeColumn);
            }

            if (mag_squared - T::one()).abs() >= T::EPSILON {
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

impl<T> Mul<Vector3<T>> for RotationMatrix<T>
where
    T: SimValue,
{
    type Output = Vector3<T>;
    fn mul(self, rhs: Vector3<T>) -> Vector3<T> {
        self.value * rhs
    }
}
