use super::{cartesian::Cartesian, cylindrical::Cylindrical};
use linear_algebra::Vector3;
use std::ops::Add;

#[derive(Debug, Default, Copy, Clone)]
pub struct Spherical {
    pub azimuth: f64,
    pub elevation: f64,
    pub radius: f64,
}

impl Spherical {
    pub fn from_vec(v: &Vector3) -> Self {
        Self {
            azimuth: v.e1,
            elevation: v.e2,
            radius: v.e3,
        }
    }
    pub fn new(azimuth: f64, elevation: f64, radius: f64) -> Self {
        Self {
            azimuth,
            elevation,
            radius,
        }
    }
}

impl From<Cartesian> for Spherical {
    fn from(cartesian: Cartesian) -> Self {
        let radius = (cartesian.x.powi(2) + cartesian.y.powi(2) + cartesian.z.powi(2)).sqrt();
        let elevation = (cartesian.z / radius).acos();
        let azimuth = cartesian.y.atan2(cartesian.x);
        Spherical::new(radius, elevation, azimuth)
    }
}

impl Add<Spherical> for Spherical {
    type Output = Self;
    fn add(self, rhs: Spherical) -> Spherical {
        Spherical::new(
            self.azimuth + rhs.azimuth,
            self.elevation + rhs.elevation,
            self.radius + rhs.radius,
        )
    }
}
