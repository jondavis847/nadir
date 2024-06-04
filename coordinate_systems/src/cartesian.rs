use super::{cylindrical::Cylindrical, spherical::Spherical};
use sim_value::SimValue;
use std::f64::consts::PI;

#[derive(Clone, Copy, Debug, Default)]
pub struct Cartesian<T>
where
    T: SimValue,
{
    x: T,
    y: T,
    z: T,
}

impl<T> Cartesian<T>
where
    T: SimValue,
{
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }
}

// Implement From<Cylindrical<T>> for Cartesian<T>
impl<T> From<Cylindrical<T>> for Cartesian<T>
where
    T: SimValue,
{
    fn from(cyl: Cylindrical<T>) -> Self {
        let height = cyl.get_height();
        let radius = cyl.get_radius();
        let theta = cyl.get_theta();

        let x = radius * (theta * T::from(PI / 180.0)).cos();
        let y = radius * (theta * T::from(PI / 180.0)).sin();
        let z = height;

        Self::new(x, y, z)
    }
}

// Implement From<Spherical<T>> for Cartesian<T>
impl<T> From<Spherical<T>> for Cartesian<T>
where
    T: SimValue,
{
    fn from(sph: Spherical<T>) -> Self {
        let azimuth = sph.get_azimuth();
        let elevation = sph.get_elevation();
        let radius = sph.get_radius();

        let x = radius * elevation.sin() * azimuth.cos();
        let y = radius * elevation.sin() * azimuth.sin();
        let z = radius * elevation.cos();

        Self::new(x, y, z)
    }
}
