use super::{matrix6::Matrix6,matrix1x6::Matrix1x6};
use rand::Rng;
use std::fmt;
use std::ops::Mul;

#[derive(Clone, Copy, Default)]
pub struct Matrix6x1 {
    pub e11: f64,
    pub e21: f64,
    pub e31: f64,
    pub e41: f64,
    pub e51: f64,
    pub e61: f64,
}

impl Matrix6x1 {
    pub fn new(e11: f64, e21: f64, e31: f64, e41: f64, e51: f64, e61: f64) -> Self {
        Self {
            e11,
            e21,
            e31,
            e41,
            e51,
            e61,
        }
    }

    pub fn rand() -> Matrix6x1 {
        let mut rng = rand::thread_rng();
        Matrix6x1 {
            e11: rng.gen(),
            e21: rng.gen(),
            e31: rng.gen(),
            e41: rng.gen(),
            e51: rng.gen(),
            e61: rng.gen(),
        }
    }

    pub fn transpose(&self) -> Matrix1x6 {
        Matrix1x6::new(self.e11,self.e21,self.e31,self.e41,self.e51,self.e61)
    }
}

impl fmt::Debug for Matrix6x1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Matrix6x1 ")?;
        writeln!(f, "   {}", self.e11)?;
        writeln!(f, "   {}", self.e21)?;
        writeln!(f, "   {}", self.e31)?;
        writeln!(f, "   {}", self.e41)?;
        writeln!(f, "   {}", self.e51)?;
        writeln!(f, "   {}", self.e61)
    }
}

impl Mul<f64> for Matrix6x1 {
    type Output = Matrix6x1;
    fn mul(self, rhs: f64) -> Self {
        Self::new(
            self.e11 * rhs,
            self.e21 * rhs,
            self.e31 * rhs,
            self.e41 * rhs,
            self.e51 * rhs,
            self.e61 * rhs,
        )
    }
}

impl Mul<Matrix1x6> for Matrix6x1 {
    type Output = Matrix6;

    fn mul(self, rhs: Matrix1x6) -> Matrix6 {
        Matrix6 {
            e11: self.e11 * rhs.e11,
            e12: self.e11 * rhs.e12,
            e13: self.e11 * rhs.e13,
            e14: self.e11 * rhs.e14,
            e15: self.e11 * rhs.e15,
            e16: self.e11 * rhs.e16,
            e21: self.e21 * rhs.e11,
            e22: self.e21 * rhs.e12,
            e23: self.e21 * rhs.e13,
            e24: self.e21 * rhs.e14,
            e25: self.e21 * rhs.e15,
            e26: self.e21 * rhs.e16,
            e31: self.e31 * rhs.e11,
            e32: self.e31 * rhs.e12,
            e33: self.e31 * rhs.e13,
            e34: self.e31 * rhs.e14,
            e35: self.e31 * rhs.e15,
            e36: self.e31 * rhs.e16,
            e41: self.e41 * rhs.e11,
            e42: self.e41 * rhs.e12,
            e43: self.e41 * rhs.e13,
            e44: self.e41 * rhs.e14,
            e45: self.e41 * rhs.e15,
            e46: self.e41 * rhs.e16,
            e51: self.e51 * rhs.e11,
            e52: self.e51 * rhs.e12,
            e53: self.e51 * rhs.e13,
            e54: self.e51 * rhs.e14,
            e55: self.e51 * rhs.e15,
            e56: self.e51 * rhs.e16,
            e61: self.e61 * rhs.e11,
            e62: self.e61 * rhs.e12,
            e63: self.e61 * rhs.e13,
            e64: self.e61 * rhs.e14,
            e65: self.e61 * rhs.e15,
            e66: self.e61 * rhs.e16,
        }
    }
}
