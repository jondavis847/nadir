#[derive(Debug, Default, Copy, Clone)]
pub struct Spherical {
    azimuth: f64,
    elevation: f64,
    radius: f64,
}

impl Spherical {
    pub fn new(azimuth: f64, elevation: f64, radius: f64) -> Self {
        Self {
            azimuth,
            elevation,
            radius,
        }
    }

    pub fn get_azimuth(&self) -> f64 {
        self.azimuth
    }

    pub fn get_elevation(&self) -> f64 {
        self.elevation
    }

    pub fn get_radius(&self) -> f64 {
        self.radius
    }
}
