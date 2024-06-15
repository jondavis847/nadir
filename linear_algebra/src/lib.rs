pub mod matrix6;
pub mod vector6;
use std::ops::{Add, Mul, Sub};
use rand::Rng;

/// A 3-dimensional vector.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Vector3 {
    pub e1: f64,
    pub e2: f64,
    pub e3: f64,
}

impl Vector3 {
    /// Creates a new `Vector3` from the given array ([f64;3]).
    ///
    /// # Arguments
    ///
    /// * `v` - An array with 3 elements all of type f64 - [f64;3].
    ///
    /// # Returns
    ///
    /// A `Vector3` instance.
    pub fn from_vec(v: [f64; 3]) -> Self {
        Self {
            e1: v[0],
            e2: v[1],
            e3: v[2],
        }
    }
    /// Creates a new `Vector3` with the given components.
    ///
    /// # Arguments
    ///
    /// * `e1` - The first component of the vector.
    /// * `e2` - The second component of the vector.
    /// * `e3` - The third component of the vector.
    ///
    /// # Returns
    ///
    /// A `Vector3` instance.
    pub fn new(e1: f64, e2: f64, e3: f64) -> Self {
        Self { e1, e2, e3 }
    }

    /// Computes the norm (magnitude) of the vector.
    ///
    /// # Returns
    ///
    /// The norm of the vector.
    pub fn norm(&self) -> f64 {
        (self.e1 * self.e1 + self.e2 * self.e2 + self.e3 * self.e3).sqrt()
    }

    /// Normalizes the vector, returning a unit vector in the same direction.
    ///
    /// # Returns
    ///
    /// A normalized `Vector3`.
    pub fn normalize(&self) -> Vector3 {
        let mag = self.norm();
        Vector3::new(self.e1 / mag, self.e2 / mag, self.e3 / mag)
    }

    /// Computes the skew-symmetric matrix of the vector.
    ///
    /// # Returns
    ///
    /// A `Matrix3` representing the skew-symmetric matrix.
    pub fn skew(&self) -> Matrix3 {
        Matrix3::new(
            0.0, self.e3, -self.e2, -self.e3, 0.0, self.e1, self.e2, -self.e1, 0.0,
        )
    }

    /// Computes the cross product of `self` and `rhs`.
    ///
    /// # Arguments
    ///
    /// * `rhs` - The right-hand side vector for the cross product.
    ///
    /// # Returns
    ///
    /// The cross product of the two vectors.
    pub fn cross(&self, rhs: Self) -> Self {
        Self::new(
            self.e2 * rhs.e3 - self.e3 * rhs.e2,
            self.e3 * rhs.e1 - self.e1 * rhs.e3,
            self.e1 * rhs.e2 - self.e2 * rhs.e1,
        )
    }
}

impl Add<Vector3> for Vector3 {
    type Output = Self;
    fn add(self, rhs: Vector3) -> Vector3 {
        Vector3::new(self.e1 + rhs.e1, self.e2 + rhs.e2, self.e3 + rhs.e3)
    }
}

impl Sub<Vector3> for Vector3 {
    type Output = Self;
    fn sub(self, rhs: Vector3) -> Vector3 {
        Vector3::new(self.e1 - rhs.e1, self.e2 - rhs.e2, self.e3 - rhs.e3)
    }
}

/// A 3x3 matrix.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Matrix3 {
    pub e11: f64,
    pub e21: f64,
    pub e31: f64,
    pub e12: f64,
    pub e22: f64,
    pub e32: f64,
    pub e13: f64,
    pub e23: f64,
    pub e33: f64,
}

impl Matrix3 {
    /// Creates a new `Matrix3` with the given elements.
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
    /// A `Matrix3` instance.
    pub fn from_vec(v: [f64; 9]) -> Self {
        Self {
            e11: v[0],
            e21: v[1],
            e31: v[2],
            e12: v[3],
            e22: v[4],
            e32: v[5],
            e13: v[6],
            e23: v[7],
            e33: v[8],
        }
    }

    /// Creates a new `Matrix3` with the given elements.
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
    /// A `Matrix3` instance.
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
    ) -> Self {
        Self {
            e11,
            e21,
            e31,
            e12,
            e22,
            e32,
            e13,
            e23,
            e33,
        }
    }
    pub fn rand() -> Matrix3 {
        let mut rng = rand::thread_rng();
        Matrix3 {
            e11: rng.gen(),
            e21: rng.gen(),
            e31: rng.gen(),
            e12: rng.gen(),
            e22: rng.gen(),
            e32: rng.gen(),
            e13: rng.gen(),
            e23: rng.gen(),
            e33: rng.gen(),
        }
    }
    pub fn transpose(&self) -> Self {
        Matrix3::new(
            self.e11, self.e12, self.e13, self.e21, self.e22, self.e23, self.e31, self.e32,
            self.e33,
        )
    }
}

impl Mul<Vector3> for Matrix3 {
    type Output = Vector3;

    /// Multiplies the matrix by a vector.
    ///
    /// # Arguments
    ///
    /// * `v` - The vector to multiply by.
    ///
    /// # Returns
    ///
    /// The result of the matrix-vector multiplication.
    fn mul(self, v: Vector3) -> Vector3 {
        Vector3::new(
            self.e11 * v.e1 + self.e12 * v.e2 + self.e13 * v.e3,
            self.e21 * v.e1 + self.e22 * v.e2 + self.e23 * v.e3,
            self.e31 * v.e1 + self.e32 * v.e2 + self.e33 * v.e3,
        )
    }
}

impl Mul<Matrix3> for Matrix3 {
    type Output = Self;

    /// Multiplies the matrix by another matrix.
    ///
    /// # Arguments
    ///
    /// * `rhs` - The right-hand side matrix to multiply by.
    ///
    /// # Returns
    ///
    /// The result of the matrix-matrix multiplication.
    fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.e11 * rhs.e11 + self.e12 * rhs.e21 + self.e13 * rhs.e31,
            self.e21 * rhs.e11 + self.e22 * rhs.e21 + self.e23 * rhs.e31,
            self.e31 * rhs.e11 + self.e32 * rhs.e21 + self.e33 * rhs.e31,
            self.e11 * rhs.e12 + self.e12 * rhs.e22 + self.e13 * rhs.e32,
            self.e21 * rhs.e12 + self.e22 * rhs.e22 + self.e23 * rhs.e32,
            self.e31 * rhs.e12 + self.e32 * rhs.e22 + self.e33 * rhs.e32,
            self.e11 * rhs.e13 + self.e12 * rhs.e23 + self.e13 * rhs.e33,
            self.e21 * rhs.e13 + self.e22 * rhs.e23 + self.e23 * rhs.e33,
            self.e31 * rhs.e13 + self.e32 * rhs.e23 + self.e33 * rhs.e33,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_vector3_new() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        assert_eq!(v.e1, 1.0);
        assert_eq!(v.e2, 2.0);
        assert_eq!(v.e3, 3.0);
    }

    #[test]
    fn test_vector3_norm() {
        let v = Vector3::new(1.0, 2.0, 2.0);
        let norm = v.norm();
        assert_relative_eq!(norm, 3.0, max_relative = 1e-12);
    }

    #[test]
    fn test_vector3_normalize() {
        let v = Vector3::new(1.0, 2.0, 2.0);
        let normalized = v.normalize();
        let norm = normalized.norm();
        assert_relative_eq!(norm, 1.0, max_relative = 1e-12);
    }

    #[test]
    fn test_vector3_skew() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let skew_matrix = v.skew();
        assert_eq!(skew_matrix.e11, 0.0);
        assert_eq!(skew_matrix.e12, -3.0);
        assert_eq!(skew_matrix.e13, 2.0);
        assert_eq!(skew_matrix.e21, 3.0);
        assert_eq!(skew_matrix.e22, 0.0);
        assert_eq!(skew_matrix.e23, -1.0);
        assert_eq!(skew_matrix.e31, -2.0);
        assert_eq!(skew_matrix.e32, 1.0);
        assert_eq!(skew_matrix.e33, 0.0);
    }

    #[test]
    fn test_vector3_cross() {
        let v1 = Vector3::new(1.0, 0.0, 0.0);
        let v2 = Vector3::new(0.0, 1.0, 0.0);
        let cross_product = v1.cross(v2);
        assert_eq!(cross_product, Vector3::new(0.0, 0.0, 1.0));
    }

    #[test]
    fn test_matrix3_new() {
        let m = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert_eq!(m.e11, 1.0);
        assert_eq!(m.e21, 2.0);
        assert_eq!(m.e31, 3.0);
        assert_eq!(m.e12, 4.0);
        assert_eq!(m.e22, 5.0);
        assert_eq!(m.e32, 6.0);
        assert_eq!(m.e13, 7.0);
        assert_eq!(m.e23, 8.0);
        assert_eq!(m.e33, 9.0);
    }

    #[test]
    fn test_matrix3_mul_vector3() {
        let m = Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
        let v = Vector3::new(1.0, 2.0, 3.0);
        let result = m * v;
        assert_eq!(result, Vector3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn test_matrix3_mul_matrix3() {
        let m1 = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let m2 = Matrix3::new(9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0);
        let result = m1 * m2;
        let expected = Matrix3::new(90.0, 114.0, 138.0, 54.0, 69.0, 84.0, 18.0, 24.0, 30.0);
        assert_eq!(result, expected);
    }
}
