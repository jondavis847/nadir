use std::{collections::HashMap, mem::take};

use crate::{
    body::{Body, BodyConnection},
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{noise::Noise, SensorModel},
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

#[typetag::serde]
impl SensorModel for Rate3Sensor {    

    fn update(&mut self, body: &Body, connection: &BodyConnection) {
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

impl MultibodyResultTrait for Rate3Sensor {
    fn get_result_entry(&mut self) -> ResultEntry {
        let mut result = HashMap::new();
        result.insert("measurement[x]".to_string(), take(&mut self.result.measurement_x));
        result.insert("measurement[y]".to_string(), take(&mut self.result.measurement_y));
        result.insert("measurement[z]".to_string(), take(&mut self.result.measurement_z));
        result.insert("noise[x]".to_string(), take(&mut self.result.noise_x));        
        result.insert("noise[y]".to_string(), take(&mut self.result.noise_y));        
        result.insert("noise[z]".to_string(), take(&mut self.result.noise_z));        
        ResultEntry::new(result)
    }

    fn initialize_result(&mut self, capacity: usize) {
        self.result.measurement_x = Vec::with_capacity(capacity);
        self.result.measurement_y = Vec::with_capacity(capacity);
        self.result.measurement_z = Vec::with_capacity(capacity);
        self.result.noise_x = Vec::with_capacity(capacity);        
        self.result.noise_y = Vec::with_capacity(capacity);        
        self.result.noise_z = Vec::with_capacity(capacity);        
    }

    fn update_result(&mut self) {
        self.result.measurement_x.push(self.state.measurement[0]);
        self.result.measurement_y.push(self.state.measurement[1]);
        self.result.measurement_z.push(self.state.measurement[2]);
        self.result.noise_x.push(self.state.noise[0]);
        self.result.noise_x.push(self.state.noise[1]);
        self.result.noise_x.push(self.state.noise[2]);
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct Rate3SensorResult {
    measurement_x: Vec<f64>,
    measurement_y: Vec<f64>,
    measurement_z: Vec<f64>,
    noise_x: Vec<f64>,
    noise_y: Vec<f64>,
    noise_z: Vec<f64>,
}