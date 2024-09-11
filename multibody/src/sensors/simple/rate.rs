use crate::{body::BodyConnection, sensors::{SensorTrait, noise::Noise}};
use rotations::RotationTrait;

/// A simple rate sensor with gaussian white noise & constant delay
/// The sensor frame is right hand rotation about X
/// The transform should put the X axis of the sensor 
/// about the desired rotation axis in the body frame
/// You can use Rotation::AlignedAxes to simplify the logic
pub struct RateSensor {        
    delay: f64,
    noise: Noise,
    value: f64, 
}

impl RateSensor {
    pub fn new(delay: f64, noise: Noise) -> Self {
        Self {            
            delay,
            noise,
            value: 0.0, // just a default value, will be updated during sim
        }
    }
}

impl SensorTrait for RateSensor {
    fn update(&mut self, connection: &BodyConnection, body: &crate::body::BodySim) {
        let rotation = connection.transform.rotation;
        let body_rate = &body.state.angular_rate_body;
        let sensor_rate = rotation.transform(*body_rate);
        self.value = sensor_rate[0]; // Always the X axis for 1d Rate Sensor
    }
}
