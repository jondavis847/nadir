use crate::{
    HardwareBuffer,
    body::BodyConnection,
    sensor::{SensorModel, noise::Noise},
};
use bytemuck::{Pod, Zeroable};
use nadir_diffeq::saving::StateWriter;
use nalgebra::Vector3;
use rotations::RotationTrait;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use uncertainty::{Normal, SimValue, Uncertainty, UncertaintyErrors};

use super::{
    SensorErrors,
    noise::{NoiseBuilder, NoiseErrors},
};

#[derive(Debug, Error)]
pub enum RateGyroErrors {
    #[error("{0}")]
    Noise(#[from] NoiseErrors),
    #[error("{0}")]
    Uncertainty(#[from] UncertaintyErrors),
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct RateGyroParametersBuilder {
    delay: Option<SimValue>,
    noise: Option<[NoiseBuilder; 3]>,
}

impl Uncertainty for RateGyroParametersBuilder {
    type Error = RateGyroErrors;
    type Output = RateGyroParameters;
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        let delay = match &self.delay {
            Some(delay) => Some(delay.sample(nominal, rng)),
            None => None,
        };
        let noise = match &self.noise {
            Some(noise) => Some([
                noise[0].sample(nominal, rng)?,
                noise[1].sample(nominal, rng)?,
                noise[2].sample(nominal, rng)?,
            ]),
            None => None,
        };
        Ok(RateGyroParameters { delay, noise })
    }
}
#[derive(Debug)]
struct RateGyroParameters {
    delay: Option<f64>,
    noise: Option<[Noise; 3]>,
}

#[derive(Debug, Default)]
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
pub struct RateGyroBuilder {
    parameters: RateGyroParametersBuilder,
}

impl RateGyroBuilder {
    pub fn new() -> Self {
        RateGyroBuilder {
            parameters: RateGyroParametersBuilder::default(),
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

    pub fn with_uncertain_delay_normal(
        mut self,
        mean: f64,
        std: f64,
    ) -> Result<Self, RateGyroErrors> {
        let dist = Normal::new(mean, std)?;
        if let Some(delay) = &mut self.parameters.delay {
            delay.set_distribution(dist.into())?;
        } else {
            self.parameters.delay = Some(SimValue::new(mean).with_distribution(dist.into())?);
        }
        Ok(self)
    }

    pub fn with_noise_normal(mut self, mean: f64, std: f64) -> Self {
        let noise1 = NoiseBuilder::new_normal(mean, std);
        let noise2 = NoiseBuilder::new_normal(mean, std);
        let noise3 = NoiseBuilder::new_normal(mean, std);
        let noise = [noise1, noise2, noise3];
        self.parameters.noise = Some(noise);
        self
    }

    pub fn set_noise_normal(&mut self, mean: f64, std: f64) {
        let noise1 = NoiseBuilder::new_normal(mean, std);
        let noise2 = NoiseBuilder::new_normal(mean, std);
        let noise3 = NoiseBuilder::new_normal(mean, std);
        let noise = [noise1, noise2, noise3];
        self.parameters.noise = Some(noise);
    }

    pub fn with_noise_uniform(mut self, low: f64, high: f64) -> Self {
        let noise1 = NoiseBuilder::new_uniform(low, high);
        let noise2 = NoiseBuilder::new_normal(low, high);
        let noise3 = NoiseBuilder::new_normal(low, high);
        let noise = [noise1, noise2, noise3];
        self.parameters.noise = Some(noise);
        self
    }

    pub fn set_noise_uniform(&mut self, low: f64, high: f64) {
        let noise1 = NoiseBuilder::new_uniform(low, high);
        let noise2 = NoiseBuilder::new_normal(low, high);
        let noise3 = NoiseBuilder::new_normal(low, high);
        let noise = [noise1, noise2, noise3];
        self.parameters.noise = Some(noise);
    }
}

impl Uncertainty for RateGyroBuilder {
    type Error = RateGyroErrors;
    type Output = RateGyro;
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        Ok(RateGyro {
            parameters: self.parameters.sample(nominal, rng)?,
            state: RateGyroState::default(),
            telemetry: RateGyroTelemetry::default(),
        })
    }
}

/// A simple rate sensor with gaussian white noise & constant delay
/// The sensor frame is right hand rotation about X
/// The transform should put the X axis of the sensor
/// about the desired rotation axis in the body frame
/// You can use Rotation::AlignedAxes to simplify the logic
#[derive(Debug)]
pub struct RateGyro {
    parameters: RateGyroParameters,
    pub state: RateGyroState,
    telemetry: RateGyroTelemetry,
}

impl SensorModel for RateGyro {
    fn update(&mut self, connection: &BodyConnection) {
        let body = connection.body.borrow();
        let transform = &connection.transform;
        let body_rate = body.state.angular_rate_body;
        let sensor_rate = transform.rotation.transform(&body_rate);
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

        //update telemetry
        self.telemetry.measurement = self.state.measurement.into();
        self.telemetry.valid = 1u8;
    }

    fn writer_save_fn(&self, writer: &mut StateWriter) {
        writer.float_buffer[0] = self.state.measurement[0];
        writer.float_buffer[1] = self.state.measurement[1];
        writer.float_buffer[2] = self.state.measurement[2];
        if let Some(noise) = &self.state.noise {
            writer.float_buffer[3] = noise[0];
            writer.float_buffer[4] = noise[1];
            writer.float_buffer[5] = noise[2];
        }
        writer.write_record().unwrap();
    }

    fn writer_headers(&self) -> &[&str] {
        if let Some(_) = &self.parameters.noise {
            &[
                "measurement[x]",
                "measurement[y]",
                "measurement[z]",
                "noise[x]",
                "noise[y]",
                "noise[z]",
            ]
        } else {
            &["measurement[x]", "measurement[y]", "measurement[z]"]
        }
    }

    fn write_buffer(&self, buffer: &mut HardwareBuffer) -> Result<(), SensorErrors> {
        buffer.write(&self.telemetry);
        Ok(())
    }
}

#[derive(Debug, Default, Copy, Clone, Pod, Zeroable)]
#[repr(C)]
pub struct RateGyroTelemetry {
    measurement: [f64; 3],
    valid: u8,
    _padding: [u8; 7],
}
