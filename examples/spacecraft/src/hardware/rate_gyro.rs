use std::{collections::HashMap, mem::take};

use multibody::{
    body::BodyConnection,
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{noise::{Noise, NoiseModels}, SensorModel},
};
use nalgebra::Vector3;
use rotations::RotationTrait;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct RateGyroParameters {
    delay: Option<f64>,
    noise: Option<[Noise; 3]>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct RateGyroState {
    noise: Option<Vector3<f64>>,
    pub measurement: Vector3<f64>,
}

/// A simple rate sensor with gaussian white noise & constant delay
/// The sensor frame is right hand rotation about X
/// The transform should put the X axis of the sensor
/// about the desired rotation axis in the body frame
/// You can use Rotation::AlignedAxes to simplify the logic
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct RateGyro {
    parameters: RateGyroParameters,
    state: RateGyroState,
    result: RateGyroResult,
}

impl RateGyro {
    pub fn new() -> Self {
        Self::default()
    }

    #[allow(dead_code)]
    pub fn with_delay(mut self, delay: f64) -> Self {
        self.parameters.delay = Some(delay);
        self
    }

    pub fn with_noise(mut self, noise: NoiseModels) -> Self {
        let noise = Noise::new(noise);
        let mut noise1 = noise.clone();
        let mut noise2 = noise.clone();
        let mut noise3 = noise.clone();
        noise1.new_seed();
        noise2.new_seed();
        noise3.new_seed();
        let noise = [noise1, noise2, noise3];
        self.parameters.noise = Some(noise);
        self
    }
}

#[typetag::serde]
impl SensorModel for RateGyro {
    fn update(&mut self, connection: &BodyConnection) {
        let body = connection.body.borrow();
        let rotation = connection.transform.rotation;
        let body_rate = body.state.angular_rate_body;
        let sensor_rate = rotation.transform(body_rate);
        if let Some(noise_model) = &mut self.parameters.noise {
            let noise1 = noise_model[0].sample();
            let noise2 = noise_model[1].sample();
            let noise3 = noise_model[2].sample();
            let noise = Vector3::new(noise1, noise2, noise3);
            self.state.noise = Some(noise);
            self.state.measurement = sensor_rate + noise;
        } else {
            self.state.measurement = sensor_rate;
        }
    }
}

impl MultibodyResultTrait for RateGyro {
    fn get_result_entry(&mut self) -> ResultEntry {
        let mut result = HashMap::new();
        result.insert(
            "measurement[x]".to_string(),
            take(&mut self.result.measurement_x),
        );
        result.insert(
            "measurement[y]".to_string(),
            take(&mut self.result.measurement_y),
        );
        result.insert(
            "measurement[z]".to_string(),
            take(&mut self.result.measurement_z),
        );
        if self.parameters.noise.is_some() {
            result.insert("noise[x]".to_string(), take(&mut self.result.noise_x));
            result.insert("noise[y]".to_string(), take(&mut self.result.noise_y));
            result.insert("noise[z]".to_string(), take(&mut self.result.noise_z));
        }
        ResultEntry::new(result)
    }

    fn initialize_result(&mut self, capacity: usize) {
        self.result.measurement_x = Vec::with_capacity(capacity);
        self.result.measurement_y = Vec::with_capacity(capacity);
        self.result.measurement_z = Vec::with_capacity(capacity);
        if self.parameters.noise.is_some() {
            self.result.noise_x = Vec::with_capacity(capacity);
            self.result.noise_y = Vec::with_capacity(capacity);
            self.result.noise_z = Vec::with_capacity(capacity);
        }
    }

    fn update_result(&mut self) {
        self.result.measurement_x.push(self.state.measurement[0]);
        self.result.measurement_y.push(self.state.measurement[1]);
        self.result.measurement_z.push(self.state.measurement[2]);
        if let Some(noise) = &self.state.noise {
            self.result.noise_x.push(noise[0]);
            self.result.noise_y.push(noise[1]);
            self.result.noise_z.push(noise[2]);
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct RateGyroResult {
    measurement_x: Vec<f64>,
    measurement_y: Vec<f64>,
    measurement_z: Vec<f64>,
    noise_x: Vec<f64>,
    noise_y: Vec<f64>,
    noise_z: Vec<f64>,
}
