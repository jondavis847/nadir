use crate::{
    body::Body,
    sensor::{
        noise::{Noise, NoiseModels},
        SensorModel,
    },
};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use time::Time;
use transforms::Transform;

use super::noise::{NoiseBuilder, NoiseModelBuilders};

#[derive(Clone, Debug, Serialize, Deserialize)]
struct GpsParametersBuilder {
    delay: Option<f64>,
    position_noise: Option<[NoiseBuilder; 3]>,
    velocity_noise: Option<[NoiseBuilder; 3]>,
}

#[derive(Debug)]
struct GpsParameters {
    delay: Option<f64>,
    position_noise: Option<[Noise; 3]>,
    velocity_noise: Option<[Noise; 3]>,
}

#[derive(Debug)]
pub struct GpsState {
    pub time: Time,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    position_noise: Option<Vector3<f64>>,
    velocity_noise: Option<Vector3<f64>>,
}

/// A simple GPS with gaussian white noise & constant delay
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GpsBuilder {
    parameters: GpsParametersBuilder,
}

/// A simple GPS with gaussian white noise & constant delay
#[derive(Debug)]
pub struct Gps {
    parameters: GpsParameters,
    pub state: GpsState,
}

impl GpsBuilder {
    pub fn new() -> Self {
        Self {
            parameters: GpsParametersBuilder {
                delay: None,
                position_noise: None,
                velocity_noise: None,
            },
        }
    }

    pub fn with_delay(mut self, delay: f64) -> Self {
        self.parameters.delay = Some(delay);
        self
    }

    pub fn with_noise_position_normal(mut self, noise: NoiseModelBuilders) -> Self {
        let noise = NoiseBuilder::new(noise);
        let mut noise1 = noise.clone();
        let mut noise2 = noise.clone();
        let mut noise3 = noise.clone();
        noise1.new_seed();
        noise2.new_seed();
        noise3.new_seed();
        let noise = [noise1, noise2, noise3];
        self.parameters.position_noise = Some(noise);
        self
    }

    pub fn with_noise_velocity(mut self, noise: NoiseModelBuilders) -> Self {
        let noise = Noise::new(noise);
        let mut noise1 = noise.clone();
        let mut noise2 = noise.clone();
        let mut noise3 = noise.clone();
        noise1.new_seed();
        noise2.new_seed();
        noise3.new_seed();
        let noise = [noise1, noise2, noise3];
        self.parameters.velocity_noise = Some(noise);
        self
    }
}

impl SensorModel for Gps {
    fn update(&mut self, body: &Body, _body_transform: &Transform) {
        let true_position = body.state.position_base;
        let true_velocity = body.state.velocity_base;

        if let Some(noise_model) = &mut self.parameters.position_noise {
            let noise1 = noise_model[0].sample();
            let noise2 = noise_model[1].sample();
            let noise3 = noise_model[2].sample();
            let noise = Vector3::new(noise1, noise2, noise3);
            self.state.position_noise = Some(noise);
            self.state.position = true_position + noise;
        } else {
            self.state.position = true_position;
        }

        if let Some(noise_model) = &mut self.parameters.velocity_noise {
            let noise1 = noise_model[0].sample();
            let noise2 = noise_model[1].sample();
            let noise3 = noise_model[2].sample();
            let noise = Vector3::new(noise1, noise2, noise3);
            self.state.velocity_noise = Some(noise);
            self.state.velocity = true_velocity + noise;
        } else {
            self.state.velocity = true_velocity;
        }
    }

    fn result_content(&self, id: u32, results: &mut nadir_result::ResultManager) {
        results.write_record(
            id,
            &[
                self.state.position[0].to_string(),
                self.state.position[1].to_string(),
                self.state.position[2].to_string(),
                self.state.velocity[0].to_string(),
                self.state.velocity[1].to_string(),
                self.state.velocity[2].to_string(),
                self.state
                    .position_noise
                    .as_ref()
                    .map_or("".to_string(), |v| v[0].to_string()),
                self.state
                    .position_noise
                    .as_ref()
                    .map_or("".to_string(), |v| v[1].to_string()),
                self.state
                    .position_noise
                    .as_ref()
                    .map_or("".to_string(), |v| v[2].to_string()),
                self.state
                    .velocity_noise
                    .as_ref()
                    .map_or("".to_string(), |v| v[0].to_string()),
                self.state
                    .velocity_noise
                    .as_ref()
                    .map_or("".to_string(), |v| v[1].to_string()),
                self.state
                    .velocity_noise
                    .as_ref()
                    .map_or("".to_string(), |v| v[2].to_string()),
            ],
        );
    }

    fn result_headers(&self) -> &[&str] {
        &[
            "position[x]",
            "position[y]",
            "position[z]",
            "velocity[x]",
            "velocity[y]",
            "velocity[z]",
            "position_noise[x]",
            "position_noise[y]",
            "position_noise[z]",
            "velocity_noise[x]",
            "velocity_noise[y]",
            "velocity_noise[z]",
        ]
    }
}
