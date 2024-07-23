use super::vector3::Vector3;
use rand::Rng;
use std::fmt;
use std::ops::{Add, Div, Mul, Neg, Sub, Index, IndexMut};
use utilities::format_number;

/// A 3x3 matrix.
#[derive(Clone, Copy, PartialEq)]
pub struct Matrix3 {
    pub e11: f64,
    pub e12: f64,
    pub e13: f64,
    pub e21: f64,
    pub e22: f64,
    pub e23: f64,
    pub e31: f64,
    pub e32: f64,
    pub e33: f64,
}

impl Matrix3 {
    pub const ZERO: Self = Self {
        e11: 0.0,
        e12: 0.0,
        e13: 0.0,
        e21: 0.0,
        e22: 0.0,
        e23: 0.0,
        e31: 0.0,
        e32: 0.0,
        e33: 0.0,
    };

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
    ) -> Self {
        Self {
            e11,
            e12,
            e13,
            e21,
            e22,
            e23,
            e31,
            e32,
            e33,
        }
    }

    pub fn rand() -> Matrix3 {
        let mut rng = rand::thread_rng();
        Matrix3 {
            e11: rng.gen(),
            e12: rng.gen(),
            e13: rng.gen(),
            e21: rng.gen(),
            e22: rng.gen(),
            e23: rng.gen(),
            e31: rng.gen(),
            e32: rng.gen(),
            e33: rng.gen(),
        }
    }

    pub fn transpose(&self) -> Self {
        Matrix3::new(
            self.e11, self.e21, self.e31, self.e12, self.e22, self.e32, self.e13, self.e23,
            self.e33,
        )
    }

    pub fn identity() -> Self {
        Matrix3::new(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
    }

    pub fn determinant(&self) -> f64 {
        self.e11 * (self.e22 * self.e33 - self.e23 * self.e32)
            - self.e12 * (self.e21 * self.e33 - self.e23 * self.e31)
            + self.e13 * (self.e21 * self.e32 - self.e22 * self.e31)
    }

    pub fn adjugate(&self) -> Self {
        Matrix3 {
            e11: self.e22 * self.e33 - self.e23 * self.e32,
            e12: self.e13 * self.e32 - self.e12 * self.e33,
            e13: self.e12 * self.e23 - self.e13 * self.e22,
            e21: self.e23 * self.e31 - self.e21 * self.e33,
            e22: self.e11 * self.e33 - self.e13 * self.e31,
            e23: self.e13 * self.e21 - self.e11 * self.e23,
            e31: self.e21 * self.e32 - self.e31 * self.e22,
            e32: self.e12 * self.e31 - self.e11 * self.e32,
            e33: self.e11 * self.e22 - self.e12 * self.e21,
        }
    }

    pub fn inv(&self) -> Option<Self> {
        let det = self.determinant();
        if det.abs() < f64::EPSILON {
            None // Matrix is not invertible
        } else {
            let adj = self.adjugate();
            Some(Matrix3 {
                e11: adj.e11 / det,
                e12: adj.e12 / det,
                e13: adj.e13 / det,
                e21: adj.e21 / det,
                e22: adj.e22 / det,
                e23: adj.e23 / det,
                e31: adj.e31 / det,
                e32: adj.e32 / det,
                e33: adj.e33 / det,
            })
        }
    }

    pub fn zeros() -> Self {
        Matrix3::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    }
}

impl Neg for Matrix3 {
    type Output = Self;
    fn neg(self) -> Self {
        Matrix3::new(
            -self.e11, -self.e12, -self.e13, -self.e21, -self.e22, -self.e23, -self.e31, -self.e32,
            -self.e33,
        )
    }
}

impl Div<f64> for Matrix3 {
    type Output = Self;
    fn div(self, f: f64) -> Self {
        Matrix3::new(
            self.e11 / f,
            self.e12 / f,
            self.e13 / f,
            self.e21 / f,
            self.e22 / f,
            self.e23 / f,
            self.e31 / f,
            self.e32 / f,
            self.e33 / f,
        )
    }
}

impl Add<Matrix3> for Matrix3 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self {
        Self::new(
            self.e11 + rhs.e11,
            self.e12 + rhs.e12,
            self.e13 + rhs.e13,
            self.e21 + rhs.e21,
            self.e22 + rhs.e22,
            self.e23 + rhs.e23,
            self.e31 + rhs.e31,
            self.e32 + rhs.e32,
            self.e33 + rhs.e33,
        )
    }
}

impl Sub<Matrix3> for Matrix3 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self {
        Self::new(
            self.e11 - rhs.e11,
            self.e12 - rhs.e12,
            self.e13 - rhs.e13,
            self.e21 - rhs.e21,
            self.e22 - rhs.e22,
            self.e23 - rhs.e23,
            self.e31 - rhs.e31,
            self.e32 - rhs.e32,
            self.e33 - rhs.e33,
        )
    }
}

impl Mul<Vector3> for Matrix3 {
    type Output = Vector3;

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

    fn mul(self, rhs: Self) -> Self {
        Self::new(
            self.e11 * rhs.e11 + self.e12 * rhs.e21 + self.e13 * rhs.e31,
            self.e11 * rhs.e12 + self.e12 * rhs.e22 + self.e13 * rhs.e32,
            self.e11 * rhs.e13 + self.e12 * rhs.e23 + self.e13 * rhs.e33,
            self.e21 * rhs.e11 + self.e22 * rhs.e21 + self.e23 * rhs.e31,
            self.e21 * rhs.e12 + self.e22 * rhs.e22 + self.e23 * rhs.e32,
            self.e21 * rhs.e13 + self.e22 * rhs.e23 + self.e23 * rhs.e33,
            self.e31 * rhs.e11 + self.e32 * rhs.e21 + self.e33 * rhs.e31,
            self.e31 * rhs.e12 + self.e32 * rhs.e22 + self.e33 * rhs.e32,
            self.e31 * rhs.e13 + self.e32 * rhs.e23 + self.e33 * rhs.e33,
        )
    }
}

impl Mul<f64> for Matrix3 {
    type Output = Self;

    fn mul(self, f: f64) -> Self {
        Matrix3::new(
            self.e11 * f,
            self.e12 * f,
            self.e13 * f,
            self.e21 * f,
            self.e22 * f,
            self.e23 * f,
            self.e31 * f,
            self.e32 * f,
            self.e33 * f,
        )
    }
}

impl Index<(usize, usize)> for Matrix3 {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        match index {
            (0, 0) => &self.e11,
            (0, 1) => &self.e12,
            (0, 2) => &self.e13,
            (1, 0) => &self.e21,
            (1, 1) => &self.e22,
            (1, 2) => &self.e23,
            (2, 0) => &self.e31,
            (2, 1) => &self.e32,
            (2, 2) => &self.e33,
            _ => panic!("invalid index - make sure 0 based"),
        }
    }
}

impl IndexMut<(usize, usize)> for Matrix3 {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        match index {
            (0, 0) => &mut self.e11,
            (0, 1) => &mut self.e12,
            (0, 2) => &mut self.e13,
            (1, 0) => &mut self.e21,
            (1, 1) => &mut self.e22,
            (1, 2) => &mut self.e23,
            (2, 0) => &mut self.e31,
            (2, 1) => &mut self.e32,
            (2, 2) => &mut self.e33,
            _ => panic!("invalid index - make sure 0 based"),
        }
    }
}

impl fmt::Debug for Matrix3 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Matrix3 ")?;
        writeln!(
            f,
            "   {}   {}   {}",
            format_number(self.e11),
            format_number(self.e12),
            format_number(self.e13)
        )?;
        writeln!(
            f,
            "   {}   {}   {}",
            format_number(self.e21),
            format_number(self.e22),
            format_number(self.e23)
        )?;
        writeln!(
            f,
            "   {}   {}   {}",
            format_number(self.e31),
            format_number(self.e32),
            format_number(self.e33)
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx_eq::assert_approx_eq;

    const TOL: f64 = 1e-12;

    fn assert_matrix3_approx_eq(m1: &Matrix3, m2: &Matrix3) {
        assert_approx_eq!(m1.e11, m2.e11, TOL);
        assert_approx_eq!(m1.e12, m2.e12, TOL);
        assert_approx_eq!(m1.e13, m2.e13, TOL);
        assert_approx_eq!(m1.e21, m2.e21, TOL);
        assert_approx_eq!(m1.e22, m2.e22, TOL);
        assert_approx_eq!(m1.e23, m2.e23, TOL);
        assert_approx_eq!(m1.e31, m2.e31, TOL);
        assert_approx_eq!(m1.e32, m2.e32, TOL);
        assert_approx_eq!(m1.e33, m2.e33, TOL);
    }

    #[test]
    fn test_matrix3_new() {
        let m = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        assert_approx_eq!(m.e11, 1.0, TOL);
        assert_approx_eq!(m.e12, 2.0, TOL);
        assert_approx_eq!(m.e13, 3.0, TOL);
        assert_approx_eq!(m.e21, 4.0, TOL);
        assert_approx_eq!(m.e22, 5.0, TOL);
        assert_approx_eq!(m.e23, 6.0, TOL);
        assert_approx_eq!(m.e31, 7.0, TOL);
        assert_approx_eq!(m.e32, 8.0, TOL);
        assert_approx_eq!(m.e33, 9.0, TOL);
    }

    #[test]
    fn test_matrix3_mul_vector3() {
        let m = Matrix3::new(2.0, 3.0, 5.0, 7.0, 11.0, 13.0, 17.0, 19.0, 23.0);
        let v = Vector3::new(29.0, 31.0, 37.0);
        let expected_e1 = 2.0 * 29.0 + 3.0 * 31.0 + 5.0 * 37.0;
        let expected_e2 = 7.0 * 29.0 + 11.0 * 31.0 + 13.0 * 37.0;
        let expected_e3 = 17.0 * 29.0 + 19.0 * 31.0 + 23.0 * 37.0;
        let result = m * v;
        assert_approx_eq!(result.e1, expected_e1, TOL);
        assert_approx_eq!(result.e2, expected_e2, TOL);
        assert_approx_eq!(result.e3, expected_e3, TOL);
    }

    #[test]
    fn test_matrix3_mul_vector3_with_identity_matrix() {
        let m = Matrix3::identity();
        let v = Vector3::new(1.0, 2.0, 3.0);
        let result = m * v;
        assert_approx_eq!(result.e1, 1.0, TOL);
        assert_approx_eq!(result.e2, 2.0, TOL);
        assert_approx_eq!(result.e3, 3.0, TOL);
    }

    #[test]
    fn test_matrix3_mul_vector3_with_zero_vector() {
        let m = Matrix3::new(2.0, 3.0, 5.0, 7.0, 11.0, 13.0, 17.0, 19.0, 23.0);
        let v = Vector3::new(0.0, 0.0, 0.0);
        let result = m * v;
        assert_approx_eq!(result.e1, 0.0, TOL);
        assert_approx_eq!(result.e2, 0.0, TOL);
        assert_approx_eq!(result.e3, 0.0, TOL);
    }

    #[test]
    fn test_matrix3_mul_vector3_with_negative_values() {
        let m = Matrix3::new(-2.0, -3.0, -5.0, -7.0, -11.0, -13.0, -17.0, -19.0, -23.0);
        let v = Vector3::new(-29.0, -31.0, -37.0);
        let expected_e1 = -2.0 * -29.0 + -3.0 * -31.0 + -5.0 * -37.0;
        let expected_e2 = -7.0 * -29.0 + -11.0 * -31.0 + -13.0 * -37.0;
        let expected_e3 = -17.0 * -29.0 + -19.0 * -31.0 + -23.0 * -37.0;
        let result = m * v;
        assert_approx_eq!(result.e1, expected_e1, TOL);
        assert_approx_eq!(result.e2, expected_e2, TOL);
        assert_approx_eq!(result.e3, expected_e3, TOL);
    }

    #[test]
    fn test_matrix3_mul_matrix3() {
        let m1 = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let m2 = Matrix3::new(9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0);
        let result = m1 * m2;
        let expected = Matrix3::new(30.0, 24.0, 18.0, 84.0, 69.0, 54.0, 138.0, 114.0, 90.0);
        assert_matrix3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_matrix3_mul_identity() {
        let m = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let identity = Matrix3::identity();
        let result = m * identity;
        let expected = m; // Multiplying by identity should yield the same matrix
        assert_matrix3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_identity_mul_matrix3() {
        let m = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let identity = Matrix3::identity();
        let result = identity * m;
        let expected = m; // Multiplying by identity should yield the same matrix
        assert_matrix3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_matrix3_mul_zero_matrix() {
        let m = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let zero = Matrix3::new(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
        let result = m * zero;
        let expected = zero; // Multiplying by zero matrix should yield a zero matrix
        assert_matrix3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_matrix3_addition() {
        let m1 = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let m2 = Matrix3::new(9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0);
        let result = m1 + m2;
        let expected = Matrix3::new(10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0);
        assert_matrix3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_matrix3_subtraction() {
        let m1 = Matrix3::new(9.0, 8.0, 7.0, 6.0, 5.0, 4.0, 3.0, 2.0, 1.0);
        let m2 = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let result = m1 - m2;
        let expected = Matrix3::new(8.0, 6.0, 4.0, 2.0, 0.0, -2.0, -4.0, -6.0, -8.0);
        assert_matrix3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_matrix3_transpose() {
        let m = Matrix3::new(1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0);
        let result = m.transpose();
        let expected = Matrix3::new(1.0, 4.0, 7.0, 2.0, 5.0, 8.0, 3.0, 6.0, 9.0);
        assert_matrix3_approx_eq(&result, &expected);
    }

    #[test]
    fn test_matrix3_inverse() {
        let m = Matrix3::new(4.0, 7.0, 2.0, 3.0, 6.0, 1.0, 2.0, 5.0, 3.0);
        let result = m.inv();
        let expected = Some(Matrix3::new(
            13.0 / 9.0,
            -11.0 / 9.0,
            -5.0 / 9.0,
            -7.0 / 9.0,
            8.0 / 9.0,
            2.0 / 9.0,
            1.0 / 3.0,
            -2.0 / 3.0,
            1.0 / 3.0,
        ));
        assert!(result.is_some());
        assert_matrix3_approx_eq(&result.unwrap(), &expected.unwrap());
    }

    #[test]
    fn test_matrix3_random() {
        let m1 = Matrix3::rand();
        let m2 = Matrix3::rand();
        // Ensure two random matrices are not equal (extremely low probability they are equal)
        assert_ne!(m1, m2);
    }
}
