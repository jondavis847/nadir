use std::ops::Add;
pub mod cartesian;
pub mod cylindrical;
pub mod spherical;

use cartesian::Cartesian;
use cylindrical::Cylindrical;
use spherical::Spherical;

#[derive(Debug, Copy, Clone)]
pub enum CoordinateSystem {
    Cartesian(Cartesian),
    Cylindrical(Cylindrical),
    Spherical(Spherical),
}

impl Default for CoordinateSystem {
    fn default() -> Self {
        Self::Cartesian(Cartesian::default())
    }
}

impl Add<CoordinateSystem> for CoordinateSystem {
    type Output = Self;
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
