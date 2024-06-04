#[derive(Debug, Default, Copy, Clone)]
pub struct Cylindrical {
    radius: f64,
    theta: f64,
    height: f64,
}

impl Cylindrical {
    pub fn new(radius:f64, theta:f64, height:f64) -> Self {
        Self {radius,theta,height}
    }
}