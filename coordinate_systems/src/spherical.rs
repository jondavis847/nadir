#[derive(Debug, Default, Copy, Clone)]
pub struct Spherical {
    radius: f64,
    azimuth: f64,
    elevation: f64,
}

impl Spherical {
    pub fn new(radius:f64, azimuth:f64, elevation:f64) -> Self {
        Self {radius,azimuth,elevation}
    }
}