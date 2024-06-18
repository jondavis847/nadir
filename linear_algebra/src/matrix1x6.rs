use super::{matrix6x1::Matrix6x1, vector6::Vector6};
use rand::Rng;
use std::fmt;
use std::ops::Mul;

#[derive(Clone, Copy, Default)]
pub struct Matrix1x6 {
    pub e11: f64,
    pub e12: f64,
    pub e13: f64,
    pub e14: f64,
    pub e15: f64,
    pub e16: f64,
}

impl Matrix1x6 {
    pub fn new(e11: f64, e12: f64, e13: f64, e14: f64, e15: f64, e16: f64) -> Self {
        Self {
            e11,
            e12,
            e13,
            e14,
            e15,
            e16,
        }
    }

    pub fn rand() -> Matrix1x6 {
        let mut rng = rand::thread_rng();
        Matrix1x6 {
            e11: rng.gen(),
            e12: rng.gen(),
            e13: rng.gen(),
            e14: rng.gen(),
            e15: rng.gen(),
            e16: rng.gen(),
        }
    }

    pub fn transpose(&self) -> Matrix6x1 {
        Matrix6x1::new(self.e11, self.e12, self.e13, self.e14, self.e15, self.e16)
    }
}

impl Mul<f64> for Matrix1x6 {
    type Output = Matrix1x6;
    fn mul(self, rhs: f64) -> Self {
        Self::new(
            self.e11 * rhs,
            self.e12 * rhs,
            self.e13 * rhs,
            self.e14 * rhs,
            self.e15 * rhs,
            self.e16 * rhs,
        )
    }
}

impl Mul<Vector6> for Matrix1x6 {
    type Output = f64;
    fn mul(self, rhs: Vector6) -> f64 {
        self.e11 * rhs.e1
            + self.e12 * rhs.e2
            + self.e13 * rhs.e3
            + self.e14 * rhs.e4
            + self.e15 * rhs.e5
            + self.e16 * rhs.e6
    }
}

impl Mul<Matrix6x1> for Matrix1x6 {
    type Output = f64;
    fn mul(self, rhs: Matrix6x1) -> f64 {
        self.e11 * rhs.e11
            + self.e12 * rhs.e21
            + self.e13 * rhs.e31
            + self.e14 * rhs.e41
            + self.e15 * rhs.e51
            + self.e16 * rhs.e61
    }
}

impl fmt::Debug for Matrix1x6 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Matrix1x6 ")?;
        writeln!(
            f,
            "   {}  {}  {}  {}  {}  {}0",
            self.e11, self.e12, self.e13, self.e14, self.e15, self.e16
        )
    }
}
