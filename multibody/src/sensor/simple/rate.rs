use crate::{
    body::{BodyConnection, BodySim},
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{noise::Noise, SensorResult, SensorTrait},
};
use polars::prelude::*;
use rotations::RotationTrait;
use serde::{Deserialize, Serialize};

use super::SimpleSensorResult;

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

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RateSensorState {
    noise: f64,
    value: f64,
}
impl SensorTrait for RateSensor {
    fn initialize_result(&self) -> SensorResult {
        SensorResult::Simple(SimpleSensorResult::Rate(RateSensorResult::default()))
    }

    fn update(&mut self, body: &BodySim, connection: &BodyConnection) {
        let rotation = connection.transform.rotation;
        let body_rate = &body.state.angular_rate_body;
        let sensor_rate = rotation.transform(*body_rate);
        self.state.noise = self.parameters.noise.sample();
        self.state.value = sensor_rate[0] + self.state.noise; // Always the X axis for 1d Rate Sensor
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RateSensorResult {
    value: Vec<f64>,
    noise: Vec<f64>,
}

impl RateSensorResult {
    pub fn update(&mut self, sensor: &RateSensor) {
        self.value.push(sensor.state.value);
        self.noise.push(sensor.state.noise);
    }
}

impl MultibodyResultTrait for RateSensorResult {
    fn add_to_dataframe(&self, df: &mut polars::prelude::DataFrame) {
        let value = Series::new("value", self.value.clone());
        let noise = Series::new("noise", self.noise.clone());

        df.with_column(value).unwrap();
        df.with_column(noise).unwrap();
    }
    fn get_state_names(&self) -> Vec<&'static str> {
        vec!["value", "noise"]
    }

    fn get_result_entry(&self) -> ResultEntry {
        ResultEntry::Sensor(SensorResult::Simple(SimpleSensorResult::Rate(self.clone())))
    }
}
