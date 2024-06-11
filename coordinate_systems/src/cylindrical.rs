use std::ops::Add;
use super::{cartesian::Cartesian, spherical::Spherical};
use linear_algebra::Vector3;

#[derive(Debug, Default, Copy, Clone)]
pub struct Cylindrical {
    pub height: f64,
    pub radius: f64,
    pub theta: f64,
}

impl Cylindrical {
    pub fn from_vec(v: &Vector3) -> Self {
        Self {
            height: v.e1,
            radius: v.e2,
            theta: v.e3,
        }
    }
    pub fn new(height: f64, radius: f64, theta: f64) -> Self {
        Self {
            height,
            radius,
            theta,
        }
    }
}

impl From<Cartesian> for Cylindrical {
    fn from(cartesian: Cartesian) -> Self {
        let r = (cartesian.x.powi(2) + cartesian.y.powi(2)).sqrt();
        let theta = cartesian.y.atan2(cartesian.x);
        Cylindrical::new(r, theta, cartesian.z)
    }
}

impl Add<Cylindrical> for Cylindrical {
    type Output = Self;
    fn add(self, rhs: Cylindrical) -> Cylindrical {
        Cylindrical::new(
            self.height + rhs.height,
            self.radius + rhs.radius,
            self.theta + rhs.theta,
        )
    }
}
