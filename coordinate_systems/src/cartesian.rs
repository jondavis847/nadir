use super::{CoordinateSystem, cylindrical::Cylindrical, spherical::Spherical};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use std::ops::{Add, Neg};

/// Represents a point in Cartesian coordinates.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct Cartesian {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl Cartesian {
    pub const ZERO: Self = Self { x: 0.0, y: 0.0, z: 0.0 };

    /// Creates a new `Cartesian` instance with the given x, y, and z components.
    ///
    /// # Arguments
    ///
    /// * `x` - The x component.
    /// * `y` - The y component.
    /// * `z` - The z component.
    ///
    /// # Returns
    ///
    /// A `Cartesian` instance.
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    /// Converts the `Cartesian` instance to a `Vector3`.
    ///
    /// # Returns
    ///
    /// A `Vector3` instance.
    pub fn vec(&self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }
}

impl From<Vector3<f64>> for Cartesian {
    /// Creates a new `Cartesian` instance from a `Vector3`.
    ///
    /// # Arguments
    ///
    /// * `v` - A reference to a `Vector3` containing the x, y, and z components.
    ///
    /// # Returns
    ///
    /// A `Cartesian` instance.
    fn from(v: Vector3<f64>) -> Cartesian {
        Cartesian::new(v[0], v[1], v[2])
    }
}

// Implement From<Cylindrical> for Cartesian
impl From<Cylindrical> for Cartesian {
    /// Converts a `Cylindrical` coordinate to a `Cartesian` coordinate.
    ///
    /// # Arguments
    ///
    /// * `cyl` - A `Cylindrical` instance.
    ///
    /// # Returns
    ///
    /// A `Cartesian` instance.
    fn from(cyl: Cylindrical) -> Self {
        let height = cyl.height;
        let radius = cyl.radius;
        let azimuth = cyl.azimuth;

        let x = radius * azimuth.cos();
        let y = radius * azimuth.sin();
        let z = height;

        Self::new(x, y, z)
    }
}

// Implement From<Spherical> for Cartesian
impl From<Spherical> for Cartesian {
    /// Converts a `Spherical` coordinate to a `Cartesian` coordinate.
    ///
    /// # Arguments
    ///
    /// * `sph` - A `Spherical` instance.
    ///
    /// # Returns
    ///
    /// A `Cartesian` instance.
    fn from(sph: Spherical) -> Self {
        let azimuth = sph.azimuth;
        let inclination = sph.inclination;
        let radius = sph.radius;

        let x = radius * inclination.sin() * azimuth.cos();
        let y = radius * inclination.sin() * azimuth.sin();
        let z = radius * inclination.cos();

        Self::new(x, y, z)
    }
}

impl From<CoordinateSystem> for Cartesian {
    /// Converts a `CoordinateSystem` enum to a `Cartesian` coordinate.
    ///
    /// # Arguments
    ///
    /// * `cs` - A `CoordinateSystem` enum instance.
    ///
    /// # Returns
    ///
    /// A `Cartesian` instance.
    fn from(cs: CoordinateSystem) -> Self {
        match cs {
            CoordinateSystem::Cartesian(cs) => cs,
            CoordinateSystem::Cylindrical(cs) => cs.into(),
            CoordinateSystem::Spherical(cs) => cs.into(),
        }
    }
}

impl Neg for Cartesian {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

impl Add<Cartesian> for Cartesian {
    type Output = Self;

    /// Adds two `Cartesian` coordinates.
    ///
    /// # Arguments
    ///
    /// * `self` - The left-hand side `Cartesian` coordinate.
    /// * `rhs` - The right-hand side `Cartesian` coordinate.
    ///
    /// # Returns
    ///
    /// A new `Cartesian` instance representing the sum.
    fn add(self, rhs: Cartesian) -> Cartesian {
        Cartesian::new(
            self.x + rhs.x,
            self.y + rhs.y,
            self.z + rhs.z,
        )
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

    /// Tests the conversion from a `Vector3` to a `Cartesian` coordinate.
    #[test]
    fn test_cartesian_from_vec() {
        let vec = Vector3::new(1.0, 2.0, 3.0);
        let cartesian = Cartesian::from(vec);
        assert_close(cartesian.x, 1.0);
        assert_close(cartesian.y, 2.0);
        assert_close(cartesian.z, 3.0);
    }

    /// Tests the conversion from a `Cylindrical` coordinate to a `Cartesian` coordinate.
    #[test]
    fn test_cartesian_from_cylindrical() {
        let cylindrical = Cylindrical { radius: 5.0, azimuth: PI / 4.0, height: 10.0 };
        let cartesian = Cartesian::from(cylindrical);

        assert_close(
            cartesian.x,
            3.5355339059327378,
        );
        assert_close(
            cartesian.y,
            3.5355339059327378,
        );
        assert_close(cartesian.z, 10.0);
    }

    /// Tests the conversion from a `Spherical` coordinate to a `Cartesian` coordinate.
    #[test]
    fn test_cartesian_from_spherical() {
        let spherical = Spherical { radius: 5.0, azimuth: PI / 4.0, inclination: PI / 4.0 };
        let cartesian = Cartesian::from(spherical);

        assert_close(cartesian.x, 2.5);
        assert_close(cartesian.y, 2.5);
        assert_close(
            cartesian.z,
            3.5355339059327378,
        );
    }

    /// Tests the addition of two `Cartesian` coordinates.
    #[test]
    fn test_cartesian_addition() {
        let cart1 = Cartesian::new(1.0, 2.0, 3.0);
        let cart2 = Cartesian::new(4.0, 5.0, 6.0);
        let result = cart1 + cart2;

        assert_close(result.x, 5.0);
        assert_close(result.y, 7.0);
        assert_close(result.z, 9.0);
    }

    /// Tests the conversion from a `CoordinateSystem` enum to a `Cartesian` coordinate.
    #[test]
    fn test_cartesian_from_coordinate_system() {
        let spherical = CoordinateSystem::Spherical(Spherical {
            radius: 5.0,
            azimuth: PI / 4.0,
            inclination: PI / 4.0,
        });
        let cartesian = Cartesian::from(spherical);

        assert_close(cartesian.x, 2.5);
        assert_close(cartesian.y, 2.5);
        assert_close(
            cartesian.z,
            3.5355339059327378,
        );
    }
}
