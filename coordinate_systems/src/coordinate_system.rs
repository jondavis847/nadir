use crate::cartesian::Cartesian;
use crate::cylindrical::Cylindrical;
use crate::spherical::Spherical;
use std::ops::Add;

/// Represents a point in a generic coordinate system, which can be Cartesian, Cylindrical, or Spherical.
#[derive(Debug, Copy, Clone)]
pub enum CoordinateSystem {
    Cartesian(Cartesian),
    Cylindrical(Cylindrical),
    Spherical(Spherical),
}

impl Default for CoordinateSystem {
    /// Provides a default value for `CoordinateSystem`, which is a default `Cartesian` coordinate.
    ///
    /// # Returns
    ///
    /// A `CoordinateSystem` instance with default `Cartesian` coordinates.
    fn default() -> Self {
        Self::Cartesian(Cartesian::default())
    }
}

impl Add<CoordinateSystem> for CoordinateSystem {
    type Output = Self;

    /// Adds two `CoordinateSystem` instances.
    ///
    /// If both are of the same type (Cartesian, Cylindrical, or Spherical), they are added directly.
    /// If they are of different types, they are converted to `Cartesian` coordinates, added, and then returned as `Cartesian`.
    ///
    /// # Arguments
    ///
    /// * `self` - The left-hand side `CoordinateSystem`.
    /// * `rhs` - The right-hand side `CoordinateSystem`.
    ///
    /// # Returns
    ///
    /// A new `CoordinateSystem` instance representing the sum.
    fn add(self, rhs: CoordinateSystem) -> CoordinateSystem {
        match (self, rhs) {
            (CoordinateSystem::Cartesian(lhs), CoordinateSystem::Cartesian(rhs)) => {
                CoordinateSystem::Cartesian(lhs + rhs)
            }
            (CoordinateSystem::Cylindrical(lhs), CoordinateSystem::Cylindrical(rhs)) => {
                CoordinateSystem::Cylindrical(lhs + rhs)
            }
            (CoordinateSystem::Spherical(lhs), CoordinateSystem::Spherical(rhs)) => {
                CoordinateSystem::Spherical(lhs + rhs)
            }
            _ => {
                let lhs = Cartesian::from(self);
                let rhs = Cartesian::from(rhs);
                CoordinateSystem::Cartesian(lhs + rhs)
            }
        }
    }
}
