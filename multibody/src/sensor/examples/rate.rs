use std::{collections::HashMap, mem::take};

use crate::{
    body::{Body, BodyConnection},
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{noise::Noise, SensorModel},
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
    result: RateSensorResult,
}

impl RateSensor {
    pub fn new(delay: f64, noise: Noise) -> Self {
        let parameters = RateSensorParameters { delay, noise };
        let state = RateSensorState::default();
        let result = RateSensorResult::default();
        Self {
            parameters,
            state,
            result,
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct RateSensorParameters {
    delay: f64,
    noise: Noise,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct RateSensorState {
    noise: f64,
    measurement: f64,
}

#[typetag::serde]
impl SensorModel for RateSensor {
    fn update(&mut self, body: &Body, connection: &BodyConnection) {
        let rotation = connection.transform.rotation;
        let body_rate = &body.state.angular_rate_body;
        let sensor_rate = rotation.transform(*body_rate);
        self.state.noise = self.parameters.noise.sample();
        self.state.measurement = sensor_rate[0] + self.state.noise; // Always the X axis for 1d Rate Sensor
    }
}

impl MultibodyResultTrait for RateSensor {
    fn get_result_entry(&mut self) -> ResultEntry {
        let mut result = HashMap::new();
        result.insert("measurement".to_string(), take(&mut self.result.measurement));
        result.insert("noise".to_string(), take(&mut self.result.noise));        
        ResultEntry::new(result)
    }

    fn initialize_result(&mut self, capacity: usize) {
        self.result.measurement = Vec::with_capacity(capacity);
        self.result.noise = Vec::with_capacity(capacity);        
    }

    fn update_result(&mut self) {
        self.result.measurement.push(self.state.measurement);
        self.result.noise.push(self.state.noise);
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RateSensorResult {
    measurement: Vec<f64>,
    noise: Vec<f64>,
}