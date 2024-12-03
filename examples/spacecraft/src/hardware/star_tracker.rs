use std::{collections::HashMap, mem::take};

use multibody::{
    body::BodyConnection,
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{
        noise::{NoiseModels, QuaternionNoise},
        SensorModel,
    },
};

use rotations::{prelude::Quaternion, Rotation};
use serde::{Deserialize, Serialize};

/// Constant parameters for the simple star tracker sensor
/// delay - a constant in seconds between truth dynamics and the sensor measurement
/// noise_(x,y,z) - noise in arcseconds for each axis
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct StarTrackerParameters {
    delay: Option<f64>,
    misalignment: Option<Rotation>,
    noise: Option<QuaternionNoise>,
}

#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct StarTrackerState {
    noise: Quaternion,
    pub measurement: Quaternion,
}

/// A star tracker attitude sensor with gaussian white noise & constant delay
/// The sensor frame is defined with +Z out the boresight, +X to the right, +Y is up
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StarTracker {
    parameters: StarTrackerParameters,
    state: StarTrackerState,
    result: StarTrackerResult,
}

impl StarTracker {
    pub fn new() -> Self {
        Self {
            parameters: StarTrackerParameters::default(),
            state: StarTrackerState::default(),
            result: StarTrackerResult::default(),
        }
    }

    #[allow(dead_code)]
    pub fn with_delay(mut self, delay: f64) -> Self {
        self.parameters.delay = Some(delay);
        self
    }

    #[allow(dead_code)]
    pub fn with_misalignment(mut self, misalignment: Rotation) -> Self {
        self.parameters.misalignment = Some(misalignment);
        self
    }

    pub fn with_noise(mut self, noise: NoiseModels) -> Self {
        let noise = QuaternionNoise::new(noise);
        self.parameters.noise = Some(noise);
        self
    }
}

#[typetag::serde]
impl SensorModel for StarTracker {
    fn update(&mut self, connection: &BodyConnection) {
        let body = connection.body.borrow();
        let body_to_st = Quaternion::from(&connection.transform.rotation);
        // Get the truth attitude
        let mut sensor_attitude = body_to_st * body.state.attitude_base;
        // Apply optional misalignment
        if let Some(misalignment) = &self.parameters.misalignment {
            sensor_attitude = Quaternion::from(misalignment) * sensor_attitude;
        }

        // Apply optional noise
        if let Some(noise) = &mut self.parameters.noise {
            let quaternion_noise = noise.sample();
            sensor_attitude = quaternion_noise * sensor_attitude;
            self.state.noise = quaternion_noise;
        } // else keep as identity from initialization
        self.state.measurement = sensor_attitude;
    }
}

impl MultibodyResultTrait for StarTracker {
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
        result.insert(
            "measurement[w]".to_string(),
            take(&mut self.result.measurement_w),
        );
        result.insert("noise[x]".to_string(), take(&mut self.result.noise_x));
        result.insert("noise[y]".to_string(), take(&mut self.result.noise_y));
        result.insert("noise[z]".to_string(), take(&mut self.result.noise_z));
        result.insert("noise[w]".to_string(), take(&mut self.result.noise_w));
        ResultEntry::new(result)
    }

    fn initialize_result(&mut self, capacity: usize) {
        self.result.measurement_x = Vec::with_capacity(capacity);
        self.result.measurement_y = Vec::with_capacity(capacity);
        self.result.measurement_z = Vec::with_capacity(capacity);
        self.result.measurement_w = Vec::with_capacity(capacity);
        self.result.noise_x = Vec::with_capacity(capacity);
        self.result.noise_y = Vec::with_capacity(capacity);
        self.result.noise_z = Vec::with_capacity(capacity);
        self.result.noise_w = Vec::with_capacity(capacity);
    }

    fn update_result(&mut self) {
        self.result.measurement_x.push(self.state.measurement.x);
        self.result.measurement_y.push(self.state.measurement.y);
        self.result.measurement_z.push(self.state.measurement.z);
        self.result.measurement_w.push(self.state.measurement.s);
        self.result.noise_x.push(self.state.noise.x);
        self.result.noise_y.push(self.state.noise.y);
        self.result.noise_z.push(self.state.noise.z);
        self.result.noise_w.push(self.state.noise.s);
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StarTrackerResult {
    measurement_x: Vec<f64>,
    measurement_y: Vec<f64>,
    measurement_z: Vec<f64>,
    measurement_w: Vec<f64>,
    noise_x: Vec<f64>,
    noise_y: Vec<f64>,
    noise_z: Vec<f64>,
    noise_w: Vec<f64>,
}
