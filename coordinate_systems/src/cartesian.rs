use super::{cylindrical::Cylindrical, spherical::Spherical};
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, Default)]
pub struct Cartesian {
    x: f64,
    y: f64,
    z: f64,
}

impl Cartesian {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn get_x(&self) -> f64 {
        self.x
    }

    pub fn get_y(&self) -> f64 {
        self.y
    }

    pub fn get_z(&self) -> f64 {
        self.z
    }
}

// Implement From<Cylindrical> for Cartesian
impl From<Cylindrical> for Cartesian {
    fn from(cyl: Cylindrical) -> Self {
        let height = cyl.get_height();
        let radius = cyl.get_radius();
        let theta = cyl.get_theta();

        let x = radius * (theta * PI / 180.0).cos();
        let y = radius * (theta * PI / 180.0).sin();
        let z = height;

        Self::new(x, y, z)
    }
}

// Implement From<Spherical> for Cartesian
impl From<Spherical> for Cartesian {
    fn from(sph: Spherical) -> Self {
        let azimuth = sph.get_azimuth();
        let elevation = sph.get_elevation();
        let radius = sph.get_radius();

        let x = radius * elevation.sin() * azimuth.cos();
        let y = radius * elevation.sin() * azimuth.sin();
        let z = radius * elevation.cos();

        Self::new(x, y, z)
    }
}
