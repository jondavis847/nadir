use crate::{
    body::BodyConnection,
    sensor::{
        noise::{Noise, NoiseBuilder},
        SensorModel,
    },
};
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use time::Time;
use uncertainty::{Normal, SimValue, Uncertainty, UncertaintyErrors};

use super::noise::NoiseErrors;

#[derive(Debug, Error)]
pub enum GpsErrors {
    #[error("gps name cannot be empty")]
    NameEmpty,
    #[error("{0}")]
    Noise(#[from] NoiseErrors),
    #[error("{0}")]
    Uncertainty(#[from] UncertaintyErrors),
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct GpsParametersBuilder {
    delay: Option<SimValue>,
    position_noise: Option<[NoiseBuilder; 3]>,
    velocity_noise: Option<[NoiseBuilder; 3]>,
}

impl Uncertainty for GpsParametersBuilder {
    type Error = GpsErrors;
    type Output = GpsParameters;
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        let delay = match &self.delay {
            Some(delay) => Some(delay.sample(nominal, rng)),
            None => None,
        };
        let position_noise = match &self.position_noise {
            Some(position_noise) => Some([
                position_noise[0].sample(nominal, rng)?,
                position_noise[1].sample(nominal, rng)?,
                position_noise[2].sample(nominal, rng)?,
            ]),
            None => None,
        };
        let velocity_noise = match &self.velocity_noise {
            Some(velocity_noise) => Some([
                velocity_noise[0].sample(nominal, rng)?,
                velocity_noise[1].sample(nominal, rng)?,
                velocity_noise[2].sample(nominal, rng)?,
            ]),
            None => None,
        };
        Ok(GpsParameters {
            delay,
            position_noise,
            velocity_noise,
        })
    }
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

impl Default for GpsState {
    fn default() -> Self {
        Self {
            time: Time::from_sec_j2k(0.0, time::TimeSystem::GPS),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            position_noise: None,
            velocity_noise: None,
        }
    }
}

/// A simple GPS with gaussian white noise & constant delay
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct GpsBuilder {
    parameters: GpsParametersBuilder,
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
        if let Some(selfdelay) = &mut self.parameters.delay {
            selfdelay.nominal = delay
        } else {
            self.parameters.delay = Some(SimValue::new(delay));
        }
        self
    }

    pub fn with_uncertain_delay_normal(mut self, mean: f64, std: f64) -> Result<Self, GpsErrors> {
        let dist = Normal::new(mean, std)?;
        if let Some(delay) = &mut self.parameters.delay {
            delay.set_distribution(dist.into())?;
        } else {
            self.parameters.delay = Some(SimValue::new(mean).with_distribution(dist.into())?);
        }
        Ok(self)
    }

    pub fn with_noise_position_normal(mut self, mean: f64, std: f64) -> Self {
        let noise1 = NoiseBuilder::new_normal(mean, std);
        let noise2 = NoiseBuilder::new_normal(mean, std);
        let noise3 = NoiseBuilder::new_normal(mean, std);
        let noise = [noise1, noise2, noise3];
        self.parameters.position_noise = Some(noise);
        self
    }

    pub fn set_noise_position_normal(&mut self, mean: f64, std: f64) {
        let noise1 = NoiseBuilder::new_normal(mean, std);
        let noise2 = NoiseBuilder::new_normal(mean, std);
        let noise3 = NoiseBuilder::new_normal(mean, std);
        let noise = [noise1, noise2, noise3];
        self.parameters.position_noise = Some(noise);
    }

    pub fn with_noise_position_uniform(mut self, low: f64, high: f64) -> Self {
        let noise1 = NoiseBuilder::new_uniform(low, high);
        let noise2 = NoiseBuilder::new_normal(low, high);
        let noise3 = NoiseBuilder::new_normal(low, high);
        let noise = [noise1, noise2, noise3];
        self.parameters.position_noise = Some(noise);
        self
    }

    pub fn set_noise_position_uniform(&mut self, low: f64, high: f64) {
        let noise1 = NoiseBuilder::new_uniform(low, high);
        let noise2 = NoiseBuilder::new_normal(low, high);
        let noise3 = NoiseBuilder::new_normal(low, high);
        let noise = [noise1, noise2, noise3];
        self.parameters.position_noise = Some(noise);
    }

    pub fn with_noise_velocity_normal(mut self, mean: f64, std: f64) -> Self {
        let noise1 = NoiseBuilder::new_normal(mean, std);
        let noise2 = NoiseBuilder::new_normal(mean, std);
        let noise3 = NoiseBuilder::new_normal(mean, std);
        let noise = [noise1, noise2, noise3];
        self.parameters.velocity_noise = Some(noise);
        self
    }

    pub fn set_noise_velocity_normal(&mut self, mean: f64, std: f64) {
        let noise1 = NoiseBuilder::new_normal(mean, std);
        let noise2 = NoiseBuilder::new_normal(mean, std);
        let noise3 = NoiseBuilder::new_normal(mean, std);
        let noise = [noise1, noise2, noise3];
        self.parameters.velocity_noise = Some(noise);
    }

    pub fn with_noise_velocity_uniform(mut self, low: f64, high: f64) -> Self {
        let noise1 = NoiseBuilder::new_uniform(low, high);
        let noise2 = NoiseBuilder::new_normal(low, high);
        let noise3 = NoiseBuilder::new_normal(low, high);
        let noise = [noise1, noise2, noise3];
        self.parameters.velocity_noise = Some(noise);
        self
    }

    pub fn set_noise_velocity_uniform(&mut self, low: f64, high: f64) {
        let noise1 = NoiseBuilder::new_uniform(low, high);
        let noise2 = NoiseBuilder::new_normal(low, high);
        let noise3 = NoiseBuilder::new_normal(low, high);
        let noise = [noise1, noise2, noise3];
        self.parameters.velocity_noise = Some(noise);
    }
}

impl Uncertainty for GpsBuilder {
    type Error = GpsErrors;
    type Output = Gps;
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        Ok(Gps {
            parameters: self.parameters.sample(nominal, rng)?,
            state: GpsState::default(),
        })
    }
}

/// A simple GPS with gaussian white noise & constant delay
#[derive(Debug)]
pub struct Gps {
    parameters: GpsParameters,
    pub state: GpsState,
}

impl SensorModel for Gps {
    fn update(&mut self, connection: &BodyConnection) {
        let body = connection.body.borrow();

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
