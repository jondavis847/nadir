use crate::{
    HardwareBuffer,
    body::BodyConnection,
    delay::DelayedValue,
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
pub enum MagnetometerErrors {
    #[error("{0}")]
    Noise(#[from] NoiseErrors),
    #[error("{0}")]
    Uncertainty(#[from] UncertaintyErrors),
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct MagnetometerParametersBuilder {
    delay: Option<SimValue>,
    noise: Option<[NoiseBuilder; 3]>,
}

impl Uncertainty for MagnetometerParametersBuilder {
    type Error = MagnetometerErrors;
    type Output = MagnetometerParameters;
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        let delay = match &self.delay {
            Some(delay) => {
                let delay = delay.sample(nominal, rng);
                Some([DelayedValue::new(delay), DelayedValue::new(delay), DelayedValue::new(delay)])
            }
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
        Ok(MagnetometerParameters { delay, noise })
    }
}

#[derive(Debug)]
struct MagnetometerParameters {
    delay: Option<[DelayedValue; 3]>,
    noise: Option<[Noise; 3]>,
}

/// A simple rate sensor with gaussian white noise & constant delay
/// The sensor frame is right hand rotation about X
/// The transform should put the X axis of the sensor
/// about the desired rotation axis in the body frame
/// You can use Rotation::AlignedAxes to simplify the logic
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct MagnetometerBuilder {
    parameters: MagnetometerParametersBuilder,
}

impl MagnetometerBuilder {
    pub fn new() -> Self {
        Self {
            parameters: MagnetometerParametersBuilder::default(),
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
    ) -> Result<Self, MagnetometerErrors> {
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

impl Uncertainty for MagnetometerBuilder {
    type Error = MagnetometerErrors;
    type Output = Magnetometer;
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        Ok(Magnetometer {
            parameters: self.parameters.sample(nominal, rng)?,
            state: MagnetometerState::default(),
            telemetry: MagnetometerTelemetry::default(),
        })
    }
}

#[derive(Debug, Default)]
pub struct MagnetometerState {
    noise: Option<Vector3<f64>>,
    pub measurement: Vector3<f64>,
}

/// A simple rate sensor with gaussian white noise & constant delay
/// The sensor frame is right hand rotation about X
/// The transform should put the X axis of the sensor
/// about the desired rotation axis in the body frame
/// You can use Rotation::AlignedAxes to simplify the logic
#[derive(Debug)]
pub struct Magnetometer {
    parameters: MagnetometerParameters,
    pub state: MagnetometerState,
    telemetry: MagnetometerTelemetry,
}

impl Magnetometer {}

impl SensorModel for Magnetometer {
    fn update(&mut self, t: f64, connection: &BodyConnection) {
        let transform = &connection.transform;
        let body = connection.body.borrow();

        let sensor_b = transform
            .rotation
            .transform(&body.state.magnetic_field_body);

        let sensor_b = if let Some(delay) = &mut self.parameters.delay {
            let new_b = transform.rotation.transform(&sensor_b);
            let mut delayed_b = Vector3::zeros();
            for i in 0..3 {
                delay[i].update(t, new_b[i]);
                delayed_b[i] = delay[i].get_delayed_reading(t);
            }
            delayed_b
        } else {
            transform.rotation.transform(&sensor_b)
        };

        if let Some(noise_model) = &mut self.parameters.noise {
            let noise1 = noise_model[0].sample();
            let noise2 = noise_model[1].sample();
            let noise3 = noise_model[2].sample();
            let noise = Vector3::new(noise1, noise2, noise3);
            self.state.noise = Some(noise);
            self.state.measurement = sensor_b + noise;
        } else {
            self.state.measurement = sensor_b
        }

        // update telemetry
        self.telemetry.measurement = self.state.measurement.into();
    }

    fn writer_save_fn(&self, writer: &mut StateWriter) {
        if let Some(noise) = &self.state.noise {
            writer.float_buffer[0] = self.state.measurement[0];
            writer.float_buffer[1] = self.state.measurement[1];
            writer.float_buffer[2] = self.state.measurement[2];
            writer.float_buffer[3] = noise[0];
            writer.float_buffer[4] = noise[1];
            writer.float_buffer[5] = noise[2];
        } else {
            writer.float_buffer[0] = self.state.measurement[0];
            writer.float_buffer[1] = self.state.measurement[1];
            writer.float_buffer[2] = self.state.measurement[2];
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
pub struct MagnetometerTelemetry {
    measurement: [f64; 3],
}
