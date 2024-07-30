use super::vector3::Vector3;
use rand::Rng;
use std::fmt;

#[derive(Clone, Copy, Default)]
pub struct Vector6 {
    pub e1: f64,
    pub e2: f64,
    pub e3: f64,
    pub e4: f64,
    pub e5: f64,
    pub e6: f64,
}

impl Vector6 {
    pub const ZERO:Vector6 = Vector6{e1:0.0, e2:0.0, e3:0.0, e4:0.0, e5:0.0, e6:0.0};
    pub fn new(e1: f64, e2: f64, e3: f64, e4: f64, e5: f64, e6: f64) -> Self {
        Self {
            e1,
            e2,
            e3,
            e4,
            e5,
            e6,
        }
    }

    #[inline]
    pub fn from_2vector3(v1: Vector3, v2: Vector3) -> Vector6 {
        Vector6 {
            e1: v1.e1,
            e2: v1.e2,
            e3: v1.e3,
            e4: v2.e1,
            e5: v2.e2,
            e6: v2.e3,
        }
    }

    pub fn rand() -> Vector6 {
        let mut rng = rand::thread_rng();
        Vector6 {
            e1: rng.gen(),
            e2: rng.gen(),
            e3: rng.gen(),
            e4: rng.gen(),
            e5: rng.gen(),
            e6: rng.gen(),
        }
    }
}

impl fmt::Debug for Vector6 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        writeln!(f, "Vector6 ")?;
        writeln!(f, "   {}", self.e1)?;
        writeln!(f, "   {}", self.e2)?;
        writeln!(f, "   {}", self.e3)?;
        writeln!(f, "   {}", self.e4)?;
        writeln!(f, "   {}", self.e5)?;
        writeln!(f, "   {}", self.e6)
    }
}
