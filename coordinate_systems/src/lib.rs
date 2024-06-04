use sim_value::SimValue;

pub mod cartesian;
pub mod cylindrical;
pub mod spherical;

use cartesian::Cartesian;
use cylindrical::Cylindrical;
use spherical::Spherical;

#[derive(Debug, Copy, Clone)]
pub enum CoordinateSystem<T>
where
    T: SimValue,
{
    Cartesian(Cartesian<T>),
    Cylindrical(Cylindrical<T>),
    Spherical(Spherical<T>),
}

impl<T> Default for CoordinateSystem<T>
where
    T: SimValue,
{
    fn default() -> Self {
        Self::Cartesian(Cartesian::default())
    }
}
