use std::f64::consts::PI;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum DipoleErrors {}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Dipole {
    m: f64,
}

impl Dipole {
    pub fn from_gh(a: f64, g: [f64; 2], h: f64) -> Result<Self, DipoleErrors> {
        let a3 = a.powi(3);
        let m = a3 * Vector3::new(g[0], g[1], h).magnitude();
        Ok(Self { m })
    }

    pub fn new(m: f64) -> Result<Self, DipoleErrors> {
        Ok(Self { m })
    }
    pub fn calculate(&self, r_ecef: &Vector3<f64>) -> Result<Vector3<f64>, DipoleErrors> {
        let x = r_ecef[0];
        let y = r_ecef[1];
        let z = r_ecef[2];
        let r = r_ecef.magnitude();
        let xy = (x * x + y * y).sqrt();

        let latgc = (z / r).asin();
        let lon = {
            let lon = if xy < 1e-9 {
                y.signum() * PI * 0.5
            } else {
                y.atan2(x)
            };
            lon % (2.0 * PI)
        };

        let m = self.m
            * Vector3::new(
                latgc.sin() * lon.cos(),
                latgc.sin() * lon.sin(),
                latgc.cos(),
            );

        let b = 3.0 * ((m.dot(r_ecef)) * r_ecef - r * r * m) / r.powi(5);
        Ok(b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use utilities::assert_equal;

    #[test]
    fn test_dipole_1() {
        let r = Vector3::new(0.0, 7e6, 0.0);

        let dipole = Dipole::from_gh(6371.2, [-29554.63, -1669.05], 5077.99).unwrap();
        let b = dipole.calculate(&r).unwrap();

        dbg!(b);

        // assert_equal(a[0], 0.0);
        // assert_equal(a[1], -1.3778135992666715e-5);
        // assert_equal(a[2], -9.808708996195295e-6);
    }
}
