use crate::{
    body::{BodyConnection, BodySim},
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{
        noise::Noise, simple::SimpleSensorResult, SensorResult, SensorTrait,
    },
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
    parameters: Rate3SensorParameters,
    state: Rate3SensorState,
    result: Rate3SensorResult,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct Rate3SensorParameters {
    delay: f64,
    noise: [Noise; 3],
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct Rate3SensorState {
    noise: Vector3<f64>,
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
        self.state.value = sensor_rate + self.state.noise;
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Rate3SensorResult {
    value_x: Vec<f64>,
    value_y: Vec<f64>,
    value_z: Vec<f64>,
    noise_x: Vec<f64>,
    noise_y: Vec<f64>,
    noise_z: Vec<f64>,
}

impl Rate3SensorResult {
    pub fn update(&mut self, sensor: &Rate3Sensor) {
        self.value_x.push(sensor.state.value[0]);
        self.value_y.push(sensor.state.value[1]);
        self.value_z.push(sensor.state.value[2]);
        self.noise_x.push(sensor.state.noise[0]);
        self.noise_y.push(sensor.state.noise[1]);
        self.noise_z.push(sensor.state.noise[2]);
    }
}

impl MultibodyResultTrait for Rate3SensorResult {
    fn get_state_names(&self) -> Vec<&'static str> {
        vec!["value", "noise"]
    }

    fn get_result_entry(&self) -> ResultEntry {
        ResultEntry::Sensor(SensorResult::Simple(SimpleSensorResult::Rate3(self.clone())))
    }
}
