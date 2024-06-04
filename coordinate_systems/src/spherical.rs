use sim_value::SimValue;

#[derive(Debug, Default, Copy, Clone)]
pub struct Spherical<T>
where
    T: SimValue,
{
    azimuth: T,
    elevation: T,
    radius: T,
}

impl<T> Spherical<T>
where
    T: SimValue,
{
    pub fn new(azimuth: T, elevation: T, radius: T) -> Self {
        Self {
            azimuth,
            elevation,
            radius,
        }
    }

    pub fn get_azimuth(&self) -> T {
        self.azimuth
    }

    pub fn get_elevation(&self) -> T {
        self.elevation
    }

    pub fn get_radius(&self) -> T {
        self.radius
    }
}
