use super::matrix3::Matrix3;
/// A 3-dimensional vector.
use rand::Rng;
use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub};
use utilities::format_number;

#[derive(Clone, Copy, Default, PartialEq)]
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
    pub fn magnitude(&self) -> f64 {
        (self.e1 * self.e1 + self.e2 * self.e2 + self.e3 * self.e3).sqrt()
    }
    pub fn dot(&self, other: Vector3) -> f64 {
        self.e1 * other.e1 + self.e2 * other.e2 + self.e3 * other.e3
    }

    /// Normalizes the vector, returning a unit vector in the same direction.
    ///
    /// # Returns
    ///
    /// A normalized `Vector3`.
    pub fn normalize(&self) -> Vector3 {
        let mag = self.magnitude();
        if mag < f64::EPSILON {
            panic!("Attempted to normalize a vector with 0 magnitude.")
        }
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

    pub fn rand() -> Vector3 {
        let mut rng = rand::thread_rng();
        Vector3 {
            e1: rng.gen(),
            e2: rng.gen(),
            e3: rng.gen(),
        }
    }
}
impl Neg for Vector3 {
    type Output = Self;

    fn neg(self) -> Self::Output {
        Self::new(-self.e1, -self.e2, -self.e3)
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

impl Mul<f64> for Vector3 {
    type Output = Self;
    fn mul(self, f: f64) -> Self {
        Vector3::new(self.e1 * f, self.e2 * f, self.e3 * f)
    }
}

impl Div<f64> for Vector3 {
    type Output = Self;
    fn div(self, f: f64) -> Self {
        Vector3::new(self.e1 / f, self.e2 / f, self.e3 / f)
    }
}

impl fmt::Debug for Vector3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Vector3 ")?;
        writeln!(f, "   {}", format_number(self.e1))?;
        writeln!(f, "   {}", format_number(self.e2))?;
        writeln!(f, "   {}", format_number(self.e3))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx_eq::assert_approx_eq;
    const TOL: f64 = 1e-12;

    fn assert_vector3_approx_eq(v1: &Vector3, v2: &Vector3) {
        assert_approx_eq!(v1.e1, v2.e1, TOL);
        assert_approx_eq!(v1.e2, v2.e2, TOL);
        assert_approx_eq!(v1.e3, v2.e3, TOL);
    }

    #[test]
    fn test_vector3_new() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        assert_approx_eq!(v.e1, 1.0, TOL);
        assert_approx_eq!(v.e2, 2.0, TOL);
        assert_approx_eq!(v.e3, 3.0, TOL);
    }

    #[test]
    fn test_vector3_addition() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        let result = v1 + v2;
        let expected = Vector3::new(5.0, 7.0, 9.0);
        assert_vector3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_vector3_subtraction() {
        let v1 = Vector3::new(4.0, 5.0, 6.0);
        let v2 = Vector3::new(1.0, 2.0, 3.0);
        let result = v1 - v2;
        let expected = Vector3::new(3.0, 3.0, 3.0);
        assert_vector3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_vector3_scalar_multiplication() {
        let v = Vector3::new(1.0, 2.0, 3.0);
        let scalar = 2.0;
        let result = v * scalar;
        let expected = Vector3::new(2.0, 4.0, 6.0);
        assert_vector3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_vector3_dot_product() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        let result = v1.dot(v2);
        let expected = 32.0; // 1*4 + 2*5 + 3*6
        assert_approx_eq!(result, expected, TOL);
    }

    #[test]
    fn test_vector3_cross_product() {
        let v1 = Vector3::new(1.0, 2.0, 3.0);
        let v2 = Vector3::new(4.0, 5.0, 6.0);
        let result = v1.cross(v2);
        let expected = Vector3::new(-3.0, 6.0, -3.0);
        assert_vector3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_vector3_magnitude() {
        let v = Vector3::new(1.0, 2.0, 2.0);
        let result = v.magnitude();
        let expected = 3.0; // sqrt(1^2 + 2^2 + 2^2)
        assert_approx_eq!(result, expected, TOL);
    }

    #[test]
    fn test_vector3_normalize() {
        let v = Vector3::new(1.0, 2.0, 2.0);
        let result = v.normalize();
        let magnitude = (1.0_f64 * 1.0 + 2.0 * 2.0 + 2.0 * 2.0).sqrt();
        let expected = Vector3::new(1.0 / magnitude, 2.0 / magnitude, 2.0 / magnitude);
        assert_vector3_approx_eq(&result, &expected);
    }

    #[test]
fn test_vector3_zero_vector() {
    let v = Vector3::new(0.0, 0.0, 0.0);
    let result = v.magnitude();
    let expected = 0.0;
    assert_approx_eq!(result, expected, TOL);
}

#[test]
fn test_vector3_negation() {
    let v = Vector3::new(1.0, -2.0, 3.0);
    let result = -v;
    let expected = Vector3::new(-1.0, 2.0, -3.0);
    assert_vector3_approx_eq(&result, &expected);
}

#[test]
fn test_vector3_scalar_division() {
    let v = Vector3::new(4.0, 8.0, 12.0);
    let scalar = 2.0;
    let result = v / scalar;
    let expected = Vector3::new(2.0, 4.0, 6.0);
    assert_vector3_approx_eq(&result, &expected);
}

#[test]
fn test_vector3_unit_vectors() {
    let x_unit = Vector3::new(1.0, 0.0, 0.0);
    let y_unit = Vector3::new(0.0, 1.0, 0.0);
    let z_unit = Vector3::new(0.0, 0.0, 1.0);
    
    assert_approx_eq!(x_unit.magnitude(), 1.0, TOL);
    assert_approx_eq!(y_unit.magnitude(), 1.0, TOL);
    assert_approx_eq!(z_unit.magnitude(), 1.0, TOL);
    
    let cross_xy = x_unit.cross(y_unit);
    let expected_cross_xy = Vector3::new(0.0, 0.0, 1.0);
    assert_vector3_approx_eq(&cross_xy, &expected_cross_xy);

    let cross_yz = y_unit.cross(z_unit);
    let expected_cross_yz = Vector3::new(1.0, 0.0, 0.0);
    assert_vector3_approx_eq(&cross_yz, &expected_cross_yz);
}
}
