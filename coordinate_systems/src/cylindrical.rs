use super::{cartesian::Cartesian, spherical::Spherical};
use linear_algebra::Vector3;
use std::ops::Add;

/// Represents a point in cylindrical coordinates.  Relative to a Cartesian x-y-z coordinate system,
/// azimuth is the right hand rotation angle about +z where +x is 0. 
/// Unique values are not enforced (all values can be negative and are unbounded). This is so 
/// that instabilites are easily detectable without rolling over.
#[derive(Debug, Default, Copy, Clone)]
pub struct Cylindrical {
    pub radius: f64,
    pub azimuth: f64,
    pub height: f64,
}

impl Cylindrical {
    /// Creates a new `Cylindrical` instance from a `Vector3`.
    ///
    /// # Arguments
    ///
    /// * `v` - A reference to a `Vector3` containing the radius, azimuth, and height.
    ///
    /// # Returns
    ///
    /// A `Cylindrical` instance.
    pub fn from_vec(v: &Vector3) -> Self {
        Self {
            radius: v.e1,
            azimuth: v.e2,
            height: v.e3,
        }
    }

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

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::PI;

    const TOL: f64 = 1e-12;

    /// Tests the conversion from a `Vector3` to a `Cylindrical` coordinate.
    #[test]
    fn test_cylindrical_from_vec() {
        let vec = Vector3::new(1.0, 2.0, 3.0);
        let cylindrical = Cylindrical::from_vec(&vec);
        let expected_radius = 1.0;
        let expected_azimuth = 2.0;
        let expected_height = 3.0;

        assert_eq!(
            cylindrical.radius, expected_radius,
            "Expected: {}, Actual: {}",
            expected_radius,
            cylindrical.radius
        );
        assert_eq!(
            cylindrical.azimuth, expected_azimuth,
            "Expected: {}, Actual: {}",
            expected_azimuth,
            cylindrical.azimuth
        );
        assert_eq!(
            cylindrical.height, expected_height,
            "Expected: {}, Actual: {}",
            expected_height,
            cylindrical.height
        );
    }

    /// Tests the conversion from a `Cartesian` coordinate to a `Cylindrical` coordinate.
    #[test]
    fn test_cylindrical_from_cartesian() {
        let cartesian = Cartesian::new(3.0, 4.0, 5.0);
        let cylindrical = Cylindrical::from(cartesian);
        let expected_radius = 5.0;
        let expected_azimuth = 0.9272952180016122;
        let expected_height = 5.0;

        assert!(
            (cylindrical.radius - expected_radius).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected_radius,
            cylindrical.radius
        );
        assert!(
            (cylindrical.azimuth - expected_azimuth).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected_azimuth,
            cylindrical.azimuth
        );
        assert!(
            (cylindrical.height - expected_height).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected_height,
            cylindrical.height
        );
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
        let expected_radius = 3.5355339059327378;
        let expected_azimuth = PI / 4.0;
        let expected_height = 3.5355339059327378;

        assert!(
            (cylindrical.radius - expected_radius).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected_radius,
            cylindrical.radius
        );
        assert!(
            (cylindrical.azimuth - expected_azimuth).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected_azimuth,
            cylindrical.azimuth
        );
        assert!(
            (cylindrical.height - expected_height).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected_height,
            cylindrical.height
        );
    }

    /// Tests the addition of two `Cylindrical` coordinates.
    #[test]
    fn test_cylindrical_addition() {
        let cyl1 = Cylindrical::new(1.0, 6.0 * PI + PI / 2.0, 1.0);
        let cyl2 = Cylindrical::new(-2.0, -10.0 * PI - PI, -2.0);
        let result = cyl1 + cyl2;

        let expected_radius = 2.23606797749979;
        let expected_azimuth = 0.4636476090008061;
        let expected_height = -1.0;

        assert!(
            (result.radius - expected_radius).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected_radius,
            result.radius
        );
        assert!(
            (result.azimuth - expected_azimuth).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected_azimuth,
            result.azimuth
        );
        assert!(
            (result.height - expected_height).abs() < TOL,
            "Expected: {}, Actual: {}",
            expected_height,
            result.height
        );
    }
}
