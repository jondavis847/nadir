use crate::CoordinateSystem;
use std::ops::Add;

use super::{cylindrical::Cylindrical, spherical::Spherical};
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, Default)]
pub struct Cartesian {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Cartesian {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }
}

// Implement From<Cylindrical> for Cartesian
impl From<Cylindrical> for Cartesian {
    fn from(cyl: Cylindrical) -> Self {
        let height = cyl.height;
        let radius = cyl.radius;
        let theta = cyl.theta;

        let x = radius * (theta * PI / 180.0).cos();
        let y = radius * (theta * PI / 180.0).sin();
        let z = height;

        Self::new(x, y, z)
    }
}

// Implement From<Spherical> for Cartesian
impl From<Spherical> for Cartesian {
    fn from(sph: Spherical) -> Self {
        let azimuth = sph.azimuth;
        let elevation = sph.elevation;
        let radius = sph.radius;

        let x = radius * elevation.sin() * azimuth.cos();
        let y = radius * elevation.sin() * azimuth.sin();
        let z = radius * elevation.cos();

        Self::new(x, y, z)
    }
}

impl From<CoordinateSystem> for Cartesian {
    fn from(cs: CoordinateSystem) -> Self {
        match cs {
            CoordinateSystem::Cartesian(cs) => cs,
            CoordinateSystem::Cylindrical(cs) => cs.into(),
            CoordinateSystem::Spherical(cs) => cs.into(),
        }
    }
}

impl Add<Cartesian> for Cartesian {
    type Output = Self;
    fn add(self, rhs: Cartesian) -> Cartesian {
        Cartesian::new(self.x + rhs.x, self.y + rhs.y, self.z + rhs.z)
    }
}
