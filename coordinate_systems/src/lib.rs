pub mod cartesian;
pub mod cylindrical;
pub mod spherical;

use cartesian::Cartesian;
use cylindrical::Cylindrical;
use spherical::Spherical;
use linear_algebra::Vector3;
use std::ops::Add;


pub mod prelude {
    pub use crate::cartesian::Cartesian;
    pub use crate::cylindrical::Cylindrical;
    pub use crate::spherical::Spherical;
    pub use crate::CoordinateSystem;
}


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

impl CoordinateSystem {
    pub fn vec(&self) -> Vector3 {
        match self {
            CoordinateSystem::Cartesian(cs) => cs.vec(),
            CoordinateSystem::Cylindrical(cs) => cs.vec(),
            CoordinateSystem::Spherical(cs) => cs.vec(),
        }
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

impl From<Cartesian> for CoordinateSystem {
    fn from(cartesian: Cartesian) -> CoordinateSystem {
        CoordinateSystem::Cartesian(cartesian)
    }
}

impl From<Cylindrical> for CoordinateSystem {
    fn from(cylindrical: Cylindrical) -> CoordinateSystem {
        CoordinateSystem::Cylindrical(cylindrical)
    }
}

impl From<Spherical> for CoordinateSystem {
    fn from(spherical: Spherical) -> CoordinateSystem {
        CoordinateSystem::Spherical(spherical)
    }
}
