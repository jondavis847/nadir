use std::ops::Mul;
use super::{Matrix3, vector6::Vector6};

#[derive(Clone, Copy, Debug, Default)]
pub struct Matrix6 {
    e11: f64,
    e12: f64,
    e13: f64,
    e14: f64,
    e15: f64,
    e16: f64,
    e21: f64,
    e22: f64,
    e23: f64,
    e24: f64,
    e25: f64,
    e26: f64,
    e31: f64,
    e32: f64,
    e33: f64,
    e34: f64,
    e35: f64,
    e36: f64,
    e41: f64,
    e42: f64,
    e43: f64,
    e44: f64,
    e45: f64,
    e46: f64,
    e51: f64,
    e52: f64,
    e53: f64,
    e54: f64,
    e55: f64,
    e56: f64,
    e61: f64,
    e62: f64,
    e63: f64,
    e64: f64,
    e65: f64,
    e66: f64,
}

impl Matrix6 {
    pub fn from_4matrix3(m1: Matrix3, m2: Matrix3, m3: Matrix3, m4: Matrix3) -> Matrix6 {
        Matrix6 {
            e11: m1.e11,
            e12: m1.e12,
            e13: m1.e13,
            e14: m2.e11,
            e15: m2.e12,
            e16: m2.e13,
            e21: m1.e21,
            e22: m1.e22,
            e23: m1.e23,
            e24: m2.e21,
            e25: m2.e22,
            e26: m2.e23,
            e31: m1.e31,
            e32: m1.e32,
            e33: m1.e33,
            e34: m2.e31,
            e35: m2.e32,
            e36: m2.e33,
            e41: m3.e11,
            e42: m3.e12,
            e43: m3.e13,
            e44: m4.e11,
            e45: m4.e12,
            e46: m4.e13,
            e51: m3.e21,
            e52: m3.e22,
            e53: m3.e23,
            e54: m4.e21,
            e55: m4.e22,
            e56: m4.e23,
            e61: m3.e31,
            e62: m3.e32,
            e63: m3.e33,
            e64: m4.e31,
            e65: m4.e32,
            e66: m4.e33,
        }
    }
}

impl Mul<Vector6> for Matrix6 {
    type Output = Vector6;
    fn mul(self, rhs: Vector6) -> Vector6 {
        Vector6 {
            e1: self.e11 * rhs.e1 + self.e12 * rhs.e2 + self.e13 * rhs.e3 + self.e14 * rhs.e4 + self.e15 * rhs.e5 + self.e16 * rhs.e6,
            e2: self.e21 * rhs.e1 + self.e22 * rhs.e2 + self.e23 * rhs.e3 + self.e24 * rhs.e4 + self.e25 * rhs.e5 + self.e26 * rhs.e6,
            e3: self.e31 * rhs.e1 + self.e32 * rhs.e2 + self.e33 * rhs.e3 + self.e34 * rhs.e4 + self.e35 * rhs.e5 + self.e36 * rhs.e6,
            e4: self.e41 * rhs.e1 + self.e42 * rhs.e2 + self.e43 * rhs.e3 + self.e44 * rhs.e4 + self.e45 * rhs.e5 + self.e46 * rhs.e6,
            e5: self.e51 * rhs.e1 + self.e52 * rhs.e2 + self.e53 * rhs.e3 + self.e54 * rhs.e4 + self.e55 * rhs.e5 + self.e56 * rhs.e6,
            e6: self.e61 * rhs.e1 + self.e62 * rhs.e2 + self.e63 * rhs.e3 + self.e64 * rhs.e4 + self.e65 * rhs.e5 + self.e66 * rhs.e6,
        }
    }
}

impl Mul<Matrix6> for Matrix6 {
    type Output = Matrix6;
    fn mul(self, rhs: Matrix6) -> Matrix6 {
        Matrix6 {
            e11: self.e11 * rhs.e11 + self.e12 * rhs.e21 + self.e13 * rhs.e31 + self.e14 * rhs.e41 + self.e15 * rhs.e51 + self.e16 * rhs.e61,
            e12: self.e11 * rhs.e12 + self.e12 * rhs.e22 + self.e13 * rhs.e32 + self.e14 * rhs.e42 + self.e15 * rhs.e52 + self.e16 * rhs.e62,
            e13: self.e11 * rhs.e13 + self.e12 * rhs.e23 + self.e13 * rhs.e33 + self.e14 * rhs.e43 + self.e15 * rhs.e53 + self.e16 * rhs.e63,
            e14: self.e11 * rhs.e14 + self.e12 * rhs.e24 + self.e13 * rhs.e34 + self.e14 * rhs.e44 + self.e15 * rhs.e54 + self.e16 * rhs.e64,
            e15: self.e11 * rhs.e15 + self.e12 * rhs.e25 + self.e13 * rhs.e35 + self.e14 * rhs.e45 + self.e15 * rhs.e55 + self.e16 * rhs.e65,
            e16: self.e11 * rhs.e16 + self.e12 * rhs.e26 + self.e13 * rhs.e36 + self.e14 * rhs.e46 + self.e15 * rhs.e56 + self.e16 * rhs.e66,

            e21: self.e21 * rhs.e11 + self.e22 * rhs.e21 + self.e23 * rhs.e31 + self.e24 * rhs.e41 + self.e25 * rhs.e51 + self.e26 * rhs.e61,
            e22: self.e21 * rhs.e12 + self.e22 * rhs.e22 + self.e23 * rhs.e32 + self.e24 * rhs.e42 + self.e25 * rhs.e52 + self.e26 * rhs.e62,
            e23: self.e21 * rhs.e13 + self.e22 * rhs.e23 + self.e23 * rhs.e33 + self.e24 * rhs.e43 + self.e25 * rhs.e53 + self.e26 * rhs.e63,
            e24: self.e21 * rhs.e14 + self.e22 * rhs.e24 + self.e23 * rhs.e34 + self.e24 * rhs.e44 + self.e25 * rhs.e54 + self.e26 * rhs.e64,
            e25: self.e21 * rhs.e15 + self.e22 * rhs.e25 + self.e23 * rhs.e35 + self.e24 * rhs.e45 + self.e25 * rhs.e55 + self.e26 * rhs.e65,
            e26: self.e21 * rhs.e16 + self.e22 * rhs.e26 + self.e23 * rhs.e36 + self.e24 * rhs.e46 + self.e25 * rhs.e56 + self.e26 * rhs.e66,

            e31: self.e31 * rhs.e11 + self.e32 * rhs.e21 + self.e33 * rhs.e31 + self.e34 * rhs.e41 + self.e35 * rhs.e51 + self.e36 * rhs.e61,
            e32: self.e31 * rhs.e12 + self.e32 * rhs.e22 + self.e33 * rhs.e32 + self.e34 * rhs.e42 + self.e35 * rhs.e52 + self.e36 * rhs.e62,
            e33: self.e31 * rhs.e13 + self.e32 * rhs.e23 + self.e33 * rhs.e33 + self.e34 * rhs.e43 + self.e35 * rhs.e53 + self.e36 * rhs.e63,
            e34: self.e31 * rhs.e14 + self.e32 * rhs.e24 + self.e33 * rhs.e34 + self.e34 * rhs.e44 + self.e35 * rhs.e54 + self.e36 * rhs.e64,
            e35: self.e31 * rhs.e15 + self.e32 * rhs.e25 + self.e33 * rhs.e35 + self.e34 * rhs.e45 + self.e35 * rhs.e55 + self.e36 * rhs.e65,
            e36: self.e31 * rhs.e16 + self.e32 * rhs.e26 + self.e33 * rhs.e36 + self.e34 * rhs.e46 + self.e35 * rhs.e56 + self.e36 * rhs.e66,

            e41: self.e41 * rhs.e11 + self.e42 * rhs.e21 + self.e43 * rhs.e31 + self.e44 * rhs.e41 + self.e45 * rhs.e51 + self.e46 * rhs.e61,
            e42: self.e41 * rhs.e12 + self.e42 * rhs.e22 + self.e43 * rhs.e32 + self.e44 * rhs.e42 + self.e45 * rhs.e52 + self.e46 * rhs.e62,
            e43: self.e41 * rhs.e13 + self.e42 * rhs.e23 + self.e43 * rhs.e33 + self.e44 * rhs.e43 + self.e45 * rhs.e53 + self.e46 * rhs.e63,
            e44: self.e41 * rhs.e14 + self.e42 * rhs.e24 + self.e43 * rhs.e34 + self.e44 * rhs.e44 + self.e45 * rhs.e54 + self.e46 * rhs.e64,
            e45: self.e41 * rhs.e15 + self.e42 * rhs.e25 + self.e43 * rhs.e35 + self.e44 * rhs.e45 + self.e45 * rhs.e55 + self.e46 * rhs.e65,
            e46: self.e41 * rhs.e16 + self.e42 * rhs.e26 + self.e43 * rhs.e36 + self.e44 * rhs.e46 + self.e45 * rhs.e56 + self.e46 * rhs.e66,

            e51: self.e51 * rhs.e11 + self.e52 * rhs.e21 + self.e53 * rhs.e31 + self.e54 * rhs.e41 + self.e55 * rhs.e51 + self.e56 * rhs.e61,
            e52: self.e51 * rhs.e12 + self.e52 * rhs.e22 + self.e53 * rhs.e32 + self.e54 * rhs.e42 + self.e55 * rhs.e52 + self.e56 * rhs.e62,
            e53: self.e51 * rhs.e13 + self.e52 * rhs.e23 + self.e53 * rhs.e33 + self.e54 * rhs.e43 + self.e55 * rhs.e53 + self.e56 * rhs.e63,
            e54: self.e51 * rhs.e14 + self.e52 * rhs.e24 + self.e53 * rhs.e34 + self.e54 * rhs.e44 + self.e55 * rhs.e54 + self.e56 * rhs.e64,
            e55: self.e51 * rhs.e15 + self.e52 * rhs.e25 + self.e53 * rhs.e35 + self.e54 * rhs.e45 + self.e55 * rhs.e55 + self.e56 * rhs.e65,
            e56: self.e51 * rhs.e16 + self.e52 * rhs.e26 + self.e53 * rhs.e36 + self.e54 * rhs.e46 + self.e55 * rhs.e56 + self.e56 * rhs.e66,

            e61: self.e61 * rhs.e11 + self.e62 * rhs.e21 + self.e63 * rhs.e31 + self.e64 * rhs.e41 + self.e65 * rhs.e51 + self.e66 * rhs.e61,
            e62: self.e61 * rhs.e12 + self.e62 * rhs.e22 + self.e63 * rhs.e32 + self.e64 * rhs.e42 + self.e65 * rhs.e52 + self.e66 * rhs.e62,
            e63: self.e61 * rhs.e13 + self.e62 * rhs.e23 + self.e63 * rhs.e33 + self.e64 * rhs.e43 + self.e65 * rhs.e53 + self.e66 * rhs.e63,
            e64: self.e61 * rhs.e14 + self.e62 * rhs.e24 + self.e63 * rhs.e34 + self.e64 * rhs.e44 + self.e65 * rhs.e54 + self.e66 * rhs.e64,
            e65: self.e61 * rhs.e15 + self.e62 * rhs.e25 + self.e63 * rhs.e35 + self.e64 * rhs.e45 + self.e65 * rhs.e55 + self.e66 * rhs.e65,
            e66: self.e61 * rhs.e16 + self.e62 * rhs.e26 + self.e63 * rhs.e36 + self.e64 * rhs.e46 + self.e65 * rhs.e56 + self.e66 * rhs.e66,
        }
    }
}

impl std::ops::Index<(usize, usize)> for Matrix6 {
    type Output = f64;

    fn index(&self, index: (usize, usize)) -> &Self::Output {
        match index {
            (0, 0) => &self.e11,
            (0, 1) => &self.e12,
            (0, 2) => &self.e13,
            (0, 3) => &self.e14,
            (0, 4) => &self.e15,
            (0, 5) => &self.e16,
            (1, 0) => &self.e21,
            (1, 1) => &self.e22,
            (1, 2) => &self.e23,
            (1, 3) => &self.e24,
            (1, 4) => &self.e25,
            (1, 5) => &self.e26,
            (2, 0) => &self.e31,
            (2, 1) => &self.e32,
            (2, 2) => &self.e33,
            (2, 3) => &self.e34,
            (2, 4) => &self.e35,
            (2, 5) => &self.e36,
            (3, 0) => &self.e41,
            (3, 1) => &self.e42,
            (3, 2) => &self.e43,
            (3, 3) => &self.e44,
            (3, 4) => &self.e45,
            (3, 5) => &self.e46,
            (4, 0) => &self.e51,
            (4, 1) => &self.e52,
            (4, 2) => &self.e53,
            (4, 3) => &self.e54,
            (4, 4) => &self.e55,
            (4, 5) => &self.e56,
            (5, 0) => &self.e61,
            (5, 1) => &self.e62,
            (5, 2) => &self.e63,
            (5, 3) => &self.e64,
            (5, 4) => &self.e65,
            (5, 5) => &self.e66,
            _ => panic!("Index out of bounds!"),
        }
    }
}

impl std::ops::IndexMut<(usize, usize)> for Matrix6 {
    fn index_mut(&mut self, index: (usize, usize)) -> &mut Self::Output {
        match index {
            (0, 0) => &mut self.e11,
            (0, 1) => &mut self.e12,
            (0, 2) => &mut self.e13,
            (0, 3) => &mut self.e14,
            (0, 4) => &mut self.e15,
            (0, 5) => &mut self.e16,
            (1, 0) => &mut self.e21,
            (1, 1) => &mut self.e22,
            (1, 2) => &mut self.e23,
            (1, 3) => &mut self.e24,
            (1, 4) => &mut self.e25,
            (1, 5) => &mut self.e26,
            (2, 0) => &mut self.e31,
            (2, 1) => &mut self.e32,
            (2, 2) => &mut self.e33,
            (2, 3) => &mut self.e34,
            (2, 4) => &mut self.e35,
            (2, 5) => &mut self.e36,
            (3, 0) => &mut self.e41,
            (3, 1) => &mut self.e42,
            (3, 2) => &mut self.e43,
            (3, 3) => &mut self.e44,
            (3, 4) => &mut self.e45,
            (3, 5) => &mut self.e46,
            (4, 0) => &mut self.e51,
            (4, 1) => &mut self.e52,
            (4, 2) => &mut self.e53,
            (4, 3) => &mut self.e54,
            (4, 4) => &mut self.e55,
            (4, 5) => &mut self.e56,
            (5, 0) => &mut self.e61,
            (5, 1) => &mut self.e62,
            (5, 2) => &mut self.e63,
            (5, 3) => &mut self.e64,
            (5, 4) => &mut self.e65,
            (5, 5) => &mut self.e66,
            _ => panic!("Index out of bounds!"),
        }
    }
}


