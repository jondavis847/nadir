use super::{cartesian::Cartesian, cylindrical::Cylindrical};
use linear_algebra::Vector3;
use std::ops::Add;

/// Represents a point in spherical coordinates. Relative to a Cartesian x-y-z coordinate system,
/// azimuth is the right hand rotation angle about +z where +x is 0, and inclination is the angle
/// from the +z axis.
/// Unique values are not enforced (all values can be negative and are unbounded). This is so
/// that instabilites are easily detectable without rolling over.
#[derive(Debug, Default, Copy, Clone)]
pub struct Spherical {
    pub radius: f64,
    pub azimuth: f64,
    pub inclination: f64,
}

impl Spherical {
    /// Creates a new `Spherical` instance from a `Vector3`.
    ///
    /// # Arguments
    ///
    /// * `v` - A reference to a `Vector3` containing the radius, azimuth, and inclination.
    ///
    /// # Returns
    ///
    /// A `Spherical` instance.
    pub fn from_vec(v: &Vector3) -> Self {
        Self {
            radius: v.e1,
            azimuth: v.e2,
            inclination: v.e3,
        }
    }

    /// Creates a new `Spherical` instance with the given radius, azimuth, and inclination.
    ///
    /// # Arguments
    ///
    /// * `radius` - The radial distance from the origin.
    /// * `azimuth` - The azimuth angle in radians.
    /// * `inclination` - The inclination angle in radians.
    ///
    /// # Returns
    ///
    /// A `Spherical` instance.
    pub fn new(radius: f64, azimuth: f64, inclination: f64) -> Self {
        Self {
            radius,
            azimuth,
            inclination,
        }
    }
}

impl From<Cartesian> for Spherical {
    /// Converts a `Cartesian` coordinate to a `Spherical` coordinate.
    ///
    /// # Arguments
    ///
    /// * `cartesian` - A `Cartesian` instance.
    ///
    /// # Returns
    ///
    /// A `Spherical` instance.
    fn from(cartesian: Cartesian) -> Self {
        let radius = (cartesian.x.powi(2) + cartesian.y.powi(2) + cartesian.z.powi(2)).sqrt();
        let azimuth = cartesian.y.atan2(cartesian.x);
        let inclination = (cartesian.z / radius).acos();
        Spherical::new(radius, azimuth, inclination)
    }
}

impl From<Cylindrical> for Spherical {
    /// Converts a `Cylindrical` coordinate to a `Spherical` coordinate.
    ///
    /// # Arguments
    ///
    /// * `cylindrical` - A `Cylindrical` instance.
    ///
    /// # Returns
    ///
    /// A `Spherical` instance.
    fn from(cylindrical: Cylindrical) -> Self {
        let radius = (cylindrical.radius.powi(2) + cylindrical.height.powi(2)).sqrt(); // r = sqrt(rho^2 + z^2)
        let inclination = (cylindrical.radius / radius).asin(); // inclination = asin(rho / r)
        Spherical::new(radius, cylindrical.azimuth, inclination)
    }
}

impl Add<Spherical> for Spherical {
    type Output = Self;

    /// Adds two `Spherical` coordinates by converting them to `Cartesian`, performing the addition, and converting back to `Spherical`.
    ///
    /// # Arguments
    ///
    /// * `self` - The left-hand side `Spherical` coordinate.
    /// * `rhs` - The right-hand side `Spherical` coordinate.
    ///
    /// # Returns
    ///
    /// A new `Spherical` instance representing the sum.
    fn add(self, rhs: Spherical) -> Spherical {
        let lhs = Cartesian::from(self);
        let rhs = Cartesian::from(rhs);
        Spherical::from(lhs + rhs)
    }
}

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

    /// Tests the conversion from a `Vector3` to a `Spherical` coordinate.
    #[test]
    fn test_spherical_from_vec() {
        let vec = Vector3::new(1.0, 2.0, 3.0);
        let spherical = Spherical::from_vec(&vec);
        assert_close(spherical.radius, 1.0);
        assert_close(spherical.azimuth, 2.0);
        assert_close(spherical.inclination, 3.0);
    }

    /// Tests the conversion from a `Cartesian` coordinate to a `Spherical` coordinate.
    #[test]
    fn test_spherical_from_cartesian() {
        let cartesian = Cartesian::new(3.0, 4.0, 5.0);
        let spherical = Spherical::from(cartesian);

        assert_close(spherical.radius, 7.0710678118654755);
        assert_close(spherical.azimuth, 0.9272952180016122);
        assert_close(spherical.inclination, 0.7853981633974483);
    }

    /// Tests the conversion from a `Cylindrical` coordinate to a `Spherical` coordinate.
    #[test]
    fn test_spherical_from_cylindrical() {
        let cylindrical = Cylindrical {
            radius: 3.0,
            azimuth: PI / 4.0,
            height: 4.0,
        };
        let spherical = Spherical::from(cylindrical);

        assert_close(spherical.radius, 5.0);
        assert_close(spherical.azimuth, PI / 4.0);
        assert_close(spherical.inclination, 0.6435011087932844);
    }

    /// Tests the addition of two `Spherical` coordinates.
    #[test]
    fn test_spherical_addition() {
        let sph1 = Spherical::new(1.0, 2.0, 3.0);
        let sph2 = Spherical::new(4.0, 5.0, 6.0);
        let result = sph1 + sph2;

        // Calculate the expected values by converting the spherical coordinates to Cartesian, adding them, and converting back
        let lhs = Cartesian::from(sph1);
        let rhs = Cartesian::from(sph2);
        let expected = Spherical::from(lhs + rhs);

        assert_close(result.radius, expected.radius);
        assert_close(result.azimuth, expected.azimuth);
        assert_close(result.inclination, expected.inclination);
    }
}
