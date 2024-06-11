use super::{cartesian::Cartesian, cylindrical::Cylindrical};
use linear_algebra::Vector3;
use std::ops::Add;

#[derive(Debug, Default, Copy, Clone)]
pub struct Spherical {
    pub radius: f64,
    pub azimuth: f64,
    pub elevation: f64,    
}

impl Spherical {
    pub fn from_vec(v: &Vector3) -> Self {
        Self {
            radius: v.e1,
            azimuth: v.e2,
            elevation: v.e3,            
        }
    }
    pub fn new(radius: f64, azimuth: f64, elevation: f64) -> Self {
        Self {
            radius,
            azimuth,
            elevation,            
        }
    }
}

impl From<Cartesian> for Spherical {
    fn from(cartesian: Cartesian) -> Self {
        let radius = (cartesian.x.powi(2) + cartesian.y.powi(2) + cartesian.z.powi(2)).sqrt();
        let azimuth = cartesian.y.atan2(cartesian.x);
        let elevation = (cartesian.z / radius).acos();        
        Spherical::new(radius, azimuth, elevation)
    }
}
impl From<Cylindrical> for Spherical {
    fn from(cylindrical: Cylindrical) -> Self {        
        let radius = (cylindrical.radius.powi(2) + cylindrical.height.powi(2)).sqrt(); // r = sqrt(rho^2 + z^2)
        let elevation = cylindrical.height.atan2(cylindrical.radius); // phi = atan2(z, rho)
        Spherical::new(radius, cylindrical.azimuth, elevation) // spherical (r, theta, phi)
    }
}
impl Add<Spherical> for Spherical {
    type Output = Self;
    fn add(self, rhs: Spherical) -> Spherical {
        Spherical::new(            
            self.radius + rhs.radius,
            self.azimuth + rhs.azimuth,
            self.elevation + rhs.elevation,
        )
    }
}
