use sim_value::SimValue;
#[derive(Debug, Default, Copy, Clone)]
pub struct Cylindrical<T>
where
    T: SimValue,
{
    height: T,
    radius: T,
    theta: T,
}

impl<T> Cylindrical<T>
where
    T: SimValue,
{
    pub fn new(height: T, radius: T, theta: T) -> Self {
        Self {
            height,
            radius,
            theta,
        }
    }

    pub fn get_height(&self) -> T {
        self.height
    }

    pub fn get_radius(&self) -> T {
        self.radius
    }

    pub fn get_theta(&self) -> T {
        self.theta
    }
}
