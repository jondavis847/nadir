#[derive(Debug, Default, Copy, Clone)]
pub struct Cylindrical {
    height: f64,
    radius: f64,
    theta: f64,
}

impl Cylindrical {
    pub fn new(height: f64, radius: f64, theta: f64) -> Self {
        Self {
            height,
            radius,
            theta,
        }
    }

    pub fn get_height(&self) -> f64 {
        self.height
    }

    pub fn get_radius(&self) -> f64 {
        self.radius
    }

    pub fn get_theta(&self) -> f64 {
        self.theta
    }
}
