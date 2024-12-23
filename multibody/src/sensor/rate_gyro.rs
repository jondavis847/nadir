use std::{fs::File, io::BufWriter};

use csv::Writer;
use crate::{
    body::BodyConnection,
    sensor::{
        noise::{Noise, NoiseModels},
        SensorModelTrait,
    },
    MultibodyResult,
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

impl SensorModelTrait for RateGyro {
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

impl MultibodyResult for RateGyro {
    fn initialize_result(&self, writer: &mut Writer<BufWriter<File>>) {
        if let Some(_) = &self.parameters.noise {
            writer
                .write_record(&[
                    "measurement[x]",
                    "measurement[y]",
                    "measurement[z]",
                    "noise[x]",
                    "noise[y]",
                    "noise[z]",
                ])
                .expect("Failed to write header");
        } else {
            writer
                .write_record(&["measurement[x]", "measurement[y]", "measurement[z]"])
                .expect("Failed to write header");
        }
    }

    fn write_result_file(&self, writer: &mut Writer<BufWriter<File>>) {
        if let Some(noise) = &self.state.noise {
            writer
                .write_record(&[
                    self.state.measurement[0].to_string(),
                    self.state.measurement[1].to_string(),
                    self.state.measurement[2].to_string(),
                    noise[0].to_string(),
                    noise[1].to_string(),
                    noise[2].to_string(),
                ])
                .expect("Failed to write rate gyro result file");
        } else {
            writer
                .write_record(&[
                    self.state.measurement[0].to_string(),
                    self.state.measurement[1].to_string(),
                    self.state.measurement[2].to_string(),
                ])
                .expect("Failed to write rate gyro result file");
        }
    }
}
