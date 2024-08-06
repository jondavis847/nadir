use super::{cartesian::Cartesian, spherical::Spherical};
use serde::{Serialize, Deserialize};
use std::ops::{Add, Neg};
use nalgebra::Vector3;
/// Represents a point in cylindrical coordinates.  Relative to a Cartesian x-y-z coordinate system,
/// azimuth is the right hand rotation angle about +z where +x is 0.
/// Unique values are not enforced (all values can be negative and are unbounded). This is so
/// that instabilites are easily detectable without rolling over.
#[derive(Debug, Default, Copy, Clone, Serialize, Deserialize)]
pub struct Cylindrical {
    pub radius: f64,
    pub azimuth: f64,
    pub height: f64,
}

impl Cylindrical {
    /// Creates a new `Cylindrical` instance with the given radius, azimuth, and height.
    ///
    /// # Arguments
    ///
    /// * `radius` - The radial distance from the origin.
    /// * `azimuth` - The azimuth angle in radians.
    /// * `height` - The height along the z-axis.
    ///
    /// # Returns
    ///
    /// A `Cylindrical` instance.
    pub fn new(radius: f64, azimuth: f64, height: f64) -> Self {
        Self {
            radius,
            azimuth,
            height,
        }
    }

    /// Converts the `Cylindrical` instance to a `Vector3`.
    ///
    /// # Returns
    ///
    /// A `Vector3` instance.
    pub fn vec(&self) -> Vector3<f64> {
        Vector3::new(self.radius, self.azimuth, self.height)
    }
}

impl From<Vector3<f64>> for Cylindrical {
    /// Creates a new `Cylindrical` instance from a `Vector3`.
    ///
    /// # Arguments
    ///
    /// * `v` - A reference to a `Vector3` containing the radius, azimuth, and height components.
    ///
    /// # Returns
    ///
    /// A `Cylindrical` instance.
    fn from(v: Vector3<f64>) -> Cylindrical {
        Cylindrical::new(v[0], v[1], v[2])
    }
}

impl From<Cartesian> for Cylindrical {
    /// Converts a `Cartesian` coordinate to a `Cylindrical` coordinate.
    ///
    /// # Arguments
    ///
    /// * `cartesian` - A `Cartesian` instance.
    ///
    /// # Returns
    ///
    /// A `Cylindrical` instance.
    fn from(cartesian: Cartesian) -> Self {
        let radius = (cartesian.x.powi(2) + cartesian.y.powi(2)).sqrt();
        let azimuth = cartesian.y.atan2(cartesian.x);
        Cylindrical::new(radius, azimuth, cartesian.z)
    }
}

impl From<Spherical> for Cylindrical {
    /// Converts a `Spherical` coordinate to a `Cylindrical` coordinate.
    ///
    /// # Arguments
    ///
    /// * `spherical` - A `Spherical` instance.
    ///
    /// # Returns
    ///
    /// A `Cylindrical` instance.
    fn from(spherical: Spherical) -> Self {
        let radius = spherical.radius * spherical.inclination.sin(); // rho = r * sin(phi)
        let height = spherical.radius * spherical.inclination.cos(); // z = r * cos(phi)
        Cylindrical::new(radius, spherical.azimuth, height) // cylindrical (rho, azimuth, z)
    }
}

// might be a faster way to do this than converting but we probably wont need to do this often
impl Neg for Cylindrical {
    type Output = Self;
    fn neg(self) -> Self {
        Cylindrical::from(Cartesian::from(self).neg())
    }
}

impl Add<Cylindrical> for Cylindrical {
    type Output = Self;

    /// Adds two `Cylindrical` coordinates by converting them to `Cartesian`, performing the addition, and converting back to `Cylindrical`.
    ///
    /// # Arguments
    ///
    /// * `self` - The left-hand side `Cylindrical` coordinate.
    /// * `rhs` - The right-hand side `Cylindrical` coordinate.
    ///
    /// # Returns
    ///
    /// A new `Cylindrical` instance representing the sum.
    fn add(self, rhs: Cylindrical) -> Cylindrical {
        let lhs = Cartesian::from(self);
        let rhs = Cartesian::from(rhs);
        Cylindrical::from(lhs + rhs)
    }
}

//TODO: make all the number unique for better tests

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-12;

    fn assert_close(actual: f64, expected: f64) {
        assert!(
            (actual - expected).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected,
            actual
        );
    }

    /// Tests the conversion from a `Vector3` to a `Cylindrical` coordinate.
    #[test]
    fn test_cylindrical_from_vec() {
        let vec = Vector3::new(1.0, 2.0, 3.0);
        let cylindrical = Cylindrical::from(vec);

        assert_close(cylindrical.radius, 1.0);
        assert_close(cylindrical.azimuth, 2.0);
        assert_close(cylindrical.height, 3.0);
    }

    /// Tests the conversion from a `Cartesian` coordinate to a `Cylindrical` coordinate.
    #[test]
    fn test_cylindrical_from_cartesian() {
        let cartesian = Cartesian::new(3.0, 4.0, 5.0);
        let cylindrical = Cylindrical::from(cartesian);

        assert_close(cylindrical.radius, 5.0);
        assert_close(cylindrical.azimuth, 0.9272952180016122);
        assert_close(cylindrical.height, 5.0);
    }

    /// Tests the conversion from a `Spherical` coordinate to a `Cylindrical` coordinate.
    #[test]
    fn test_cylindrical_from_spherical() {
        let spherical = Spherical {
            radius: 5.0,
            azimuth: PI / 4.0,
            inclination: PI / 4.0,
        };
        let cylindrical = Cylindrical::from(spherical);

        assert_close(cylindrical.radius, 3.5355339059327378);
        assert_close(cylindrical.azimuth, PI / 4.0);
        assert_close(cylindrical.height, 3.5355339059327378);
    }

    /// Tests the addition of two `Cylindrical` coordinates.
    #[test]
    fn test_cylindrical_addition() {
        let cyl1 = Cylindrical::new(1.0, 6.0 * PI + PI / 2.0, 1.0);
        let cyl2 = Cylindrical::new(-2.0, -10.0 * PI - PI, -2.0);
        let result = cyl1 + cyl2;

        assert_close(result.radius, 2.23606797749979);
        assert_close(result.azimuth, 0.4636476090008061);
        assert_close(result.height, -1.0);
    }
}
