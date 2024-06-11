use super::{cartesian::Cartesian, spherical::Spherical};
use linear_algebra::Vector3;
use std::ops::Add;

#[derive(Debug, Default, Copy, Clone)]
pub struct Cylindrical {
    pub radius: f64,
    pub azimuth: f64,
    pub height: f64,
}

impl Cylindrical {
    pub fn from_vec(v: &Vector3) -> Self {
        Self {
            radius: v.e1,
            azimuth: v.e2,
            height: v.e3,
        }
    }
    pub fn new(radius: f64, azimuth: f64, height: f64) -> Self {
        Self {
            radius,
            azimuth,
            height,
        }
    }
}

impl From<Cartesian> for Cylindrical {
    fn from(cartesian: Cartesian) -> Self {
        let radius = (cartesian.x.powi(2) + cartesian.y.powi(2)).sqrt();
        let azimuth = cartesian.y.atan2(cartesian.x);
        Cylindrical::new(radius,azimuth,cartesian.z)
    }
}

impl From<Spherical> for Cylindrical {
    fn from(spherical: Spherical) -> Self {
        let radius = spherical.radius * spherical.elevation.cos(); // rho = r * sin(phi)
        let height = spherical.radius * spherical.elevation.sin(); // z = r * cos(phi)
        Cylindrical::new(radius,spherical.azimuth,height) // cylindrical (rho, azimuth, z)
    }
}

impl Add<Cylindrical> for Cylindrical {
    type Output = Self;
    fn add(self, rhs: Cylindrical) -> Cylindrical {
        Cylindrical::new(            
            self.radius + rhs.radius,
            self.azimuth + rhs.azimuth,
            self.height + rhs.height,
        )
    }
}
