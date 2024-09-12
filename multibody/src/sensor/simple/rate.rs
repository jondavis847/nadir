use crate::{
    body::{BodyConnection, BodySim},
    result::MultibodyResultTrait,
    sensor::{noise::Noise, SensorTrait},
};
use rotations::RotationTrait;
use serde::{Deserialize, Serialize};

/// A simple rate sensor with gaussian white noise & constant delay
/// The sensor frame is right hand rotation about X
/// The transform should put the X axis of the sensor
/// about the desired rotation axis in the body frame
/// You can use Rotation::AlignedAxes to simplify the logic

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RateSensor {
    parameters: RateSensorParameters,
    state: RateSensorState,
}

impl RateSensor {
    pub fn new(delay: f64, noise: Noise) -> Self {
        let parameters = RateSensorParameters { delay, noise };
        let state = RateSensorState::default();
        Self { parameters, state }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RateSensorParameters {
    delay: f64,
    noise: Noise,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RateSensorState {
    noise: f64,
    value: f64,
}
impl SensorTrait for RateSensor {
    fn update(&mut self, body: &BodySim, connection: &BodyConnection) {
        let rotation = connection.transform.rotation;
        let body_rate = &body.state.angular_rate_body;
        let sensor_rate = rotation.transform(*body_rate);
        self.state.noise = self.parameters.noise.sample();
        self.state.value = sensor_rate[0] + self.state.noise; // Always the X axis for 1d Rate Sensor
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RateSensorResult(pub Vec<RateSensorState>);

impl MultibodyResultTrait for RateSensorResult {
    fn get_state_names(&self) -> Vec<&'static str> {
        vec!["value","noise"]
    }
}
