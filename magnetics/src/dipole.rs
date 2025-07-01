use std::f64::consts::PI;

use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum DipoleErrors {}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Dipole {
    m: Vector3<f64>,
    offset: Vector3<f64>, // cartesian offset along dipole axis
}

impl Dipole {
    pub fn new(a: f64, m: f64, lat: f64, lon: f64, offset: f64) -> Result<Self, DipoleErrors> {
        // convert lat and lon to rad
        let lat = lat * PI / 180.0;
        let lon = lon * PI / 180.0;
        // convert from guass to Am2
        let guass_to_tesla = 1e-4;
        let tesla_to_am2 = a.powi(3);
        let m = m * guass_to_tesla * tesla_to_am2;
        // this is from markley/crassidis, but with lat being -pi/2 to pi/2 rather then 0 to pi
        // gsfc planetary constants are given in -pi/2 to pi/2
        let m = m * Vector3::new(
            -lat.cos() * lon.cos(),
            -lat.cos() * lon.sin(),
            -lat.sin(),
        );
        // get the unit vector along dipole axis
        let u = Vector3::new(
            lat.cos() * lon.cos(),
            lat.cos() * lon.sin(),
            lat.sin(),
        )
        .normalize();
        // calculate cartesian offset vector
        let offset = offset * a * u;
        Ok(Self { m, offset })
    }

    pub fn from_gh(a: f64, g: [f64; 2], h: f64) -> Result<Self, DipoleErrors> {
        let nt_to_tesla = 1e-9;
        let tesla_to_am2 = a.powi(3);
        let m = nt_to_tesla * tesla_to_am2 * Vector3::new(g[1], h, g[0]);
        Ok(Self {
            m,
            offset: Vector3::zeros(), //TODO: add in offset?
        })
    }

    pub fn calculate(&self, r_ecef: &Vector3<f64>) -> Result<Vector3<f64>, DipoleErrors> {
        let r_dipole = r_ecef - self.offset;
        let r = r_dipole.magnitude();
        let b = (3.0
            * self
                .m
                .dot(&r_dipole)
            * r_dipole
            - r.powi(2) * self.m)
            / r.powi(5);
        // convert back to nT
        Ok(b * 1e9)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use utilities::{assert_equal, assert_equal_reltol};

    #[test]
    fn test_dipole_gh() {
        let r = Vector3::new(7e6, 0.0, 0.0);

        let dipole = Dipole::from_gh(
            6.3712e6,
            [-29554.63, -1669.05],
            5077.99,
        )
        .unwrap();
        let b = dipole
            .calculate(&r)
            .unwrap();

        assert_equal(b[0], -2516.9172529558114);
        assert_equal(b[1], -3828.7890240966663);
        assert_equal(b[2], 22284.101180829042);
    }

    #[test]
    fn test_dipole() {
        let r = Vector3::new(7e6, 0.0, 0.0);

        let dipole = Dipole::new(
            6.3712e6, 0.306, 80.65, -72.68, 0.076,
        )
        .unwrap();
        let b = dipole
            .calculate(&r)
            .unwrap();

        // make sure mlatlon is reasonably close ( ~10% ) within gh
        assert_equal_reltol(b[0], -2516.9172529558114, 0.2);
        assert_equal_reltol(b[1], -3828.7890240966663, 0.2);
        assert_equal_reltol(b[2], 22284.101180829042, 0.1);
    }
}
