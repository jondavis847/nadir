use std::collections::HashMap;

use crate::{
    body::{BodyConnection, BodySim},
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{noise::Noise, simple::SimpleSensorResult, SensorResult, SensorTrait}, MultibodyErrors,
};
use nalgebra::Vector3;
use rotations::RotationTrait;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
struct Rate3SensorParameters {
    delay: f64,
    noise: [Noise; 3],
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct Rate3SensorState {
    noise: Vector3<f64>,
    pub measurement: Vector3<f64>,
}

/// A simple rate sensor with gaussian white noise & constant delay
/// The sensor frame is right hand rotation about X
/// The transform should put the X axis of the sensor
/// about the desired rotation axis in the body frame
/// You can use Rotation::AlignedAxes to simplify the logic
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Rate3Sensor {
    parameters: Rate3SensorParameters,
    state: Rate3SensorState,
    result: Rate3SensorResult,
}

impl Rate3Sensor {
    pub fn new(delay: f64, noise: Noise) -> Self {
        let mut noise1 = noise.clone();
        let mut noise2 = noise.clone();
        let mut noise3 = noise.clone();
        noise1.new_seed();
        noise2.new_seed();
        noise3.new_seed();
        let noise = [noise1, noise2, noise3];

        let parameters = Rate3SensorParameters { delay, noise };
        let state = Rate3SensorState::default();
        let result = Rate3SensorResult::default();

        Self {
            parameters,
            state,
            result,
        }
    }
}

impl SensorTrait for Rate3Sensor {
    fn initialize_result(&self) -> SensorResult {
        SensorResult::Simple(SimpleSensorResult::Rate3(Rate3SensorResult::default()))
    }

    fn update(&mut self, body: &BodySim, connection: &BodyConnection) {
        let rotation = connection.transform.rotation;
        let body_rate = &body.state.angular_rate_body;
        let sensor_rate = rotation.transform(*body_rate);
        let noise1 = self.parameters.noise[0].sample();
        let noise2 = self.parameters.noise[1].sample();
        let noise3 = self.parameters.noise[2].sample();
        self.state.noise = Vector3::new(noise1, noise2, noise3);
        self.state.measurement = sensor_rate + self.state.noise;
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Rate3SensorResult(HashMap<String, Vec<f64>>);

impl Rate3SensorResult {
    pub fn new() -> Self {
        let mut result = HashMap::new();
        Self::STATES.iter().for_each(|state| {
            result.insert(state.to_string(), Vec::new());
        });
        Self(result)
    }
}

impl MultibodyResultTrait for Rate3SensorResult {
    type Component = Rate3Sensor;
    const STATES: &'static [&'static str] = &[
        "measurement[x]",
        "measurement[y]",
        "measurement[z]",
        "noise[x]",
        "noise[y]",
        "noise[z]",
    ];

    fn get_result_entry(&self) -> ResultEntry {
        ResultEntry::Sensor(SensorResult::Simple(SimpleSensorResult::Rate3(
            self.clone(),
        )))
    }
    
    fn get_state_value(&self, state: &str) -> Result<&Vec<f64>, MultibodyErrors> {
        self.0.get(state).ok_or(MultibodyErrors::ComponentStateNotFound(state.to_string()))
    }

    fn get_state_names(&self) -> &'static [&'static str] {
        Self::STATES
    }

    fn update(&mut self, sensor: &Rate3Sensor) {
        self.0
            .get_mut("measurement[x]")
            .unwrap()
            .push(sensor.state.measurement[0]);
        self.0
            .get_mut("measurement[y]")
            .unwrap()
            .push(sensor.state.measurement[1]);
        self.0
            .get_mut("measurement[z]")
            .unwrap()
            .push(sensor.state.measurement[2]);
        self.0
            .get_mut("noise[x]")
            .unwrap()
            .push(sensor.state.noise[0]);
        self.0
            .get_mut("noise[y]")
            .unwrap()
            .push(sensor.state.noise[1]);
        self.0
            .get_mut("noise[z]")
            .unwrap()
            .push(sensor.state.noise[2]);
    }
}
