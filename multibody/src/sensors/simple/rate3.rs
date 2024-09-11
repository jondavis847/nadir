use crate::{
    body::{BodyConnection, BodySim},
    sensors::{noise::Noise, SensorTrait},
};
use nalgebra::Vector3;
use rotations::RotationTrait;
use serde::{Deserialize, Serialize};

/// A simple rate sensor with gaussian white noise & constant delay
/// The sensor frame is right hand rotation about X
/// The transform should put the X axis of the sensor
/// about the desired rotation axis in the body frame
/// You can use Rotation::AlignedAxes to simplify the logic
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Rate3Sensor {    
    delay: f64,
    noise: [Noise; 3],
    value: Vector3<f64>,
}

impl Rate3Sensor {
    pub fn new(delay: f64, noise: Noise) -> Self {
        let mut noise1 = noise.clone();
        let mut noise2 = noise.clone();
        let mut noise3 = noise.clone();
        noise1.new_seed();
        noise2.new_seed();
        noise3.new_seed();

        Self {            
            delay,
            noise: [noise1, noise2, noise3],
            value: Vector3::zeros(), // just a default value, will be updated during sim
        }
    }
}

impl SensorTrait for Rate3Sensor {
    fn update(&mut self, connection: &BodyConnection, body: &BodySim) {
        let rotation = connection.transform.rotation;
        let body_rate = &body.state.angular_rate_body;
        let sensor_rate = rotation.transform(*body_rate);
        self.value = sensor_rate;
    }
}
