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
