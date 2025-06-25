use super::{
    SensorErrors,
    noise::{NoiseErrors, QuaternioNoiseBuilder},
};
use crate::{
    HardwareBuffer,
    body::BodyConnection,
    delay::DelayedQuaternion,
    sensor::{SensorModel, noise::QuaternionNoise},
};
use bytemuck::{Pod, Zeroable};
use nadir_diffeq::saving::StateWriter;
use rotations::prelude::{QuaternionErrors, UnitQuaternion, UnitQuaternionBuilder};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uncertainty::{UncertainValue, Uncertainty, UncertaintyErrors};

#[derive(Debug, Error)]
pub enum StarTrackerErrors {
    #[error("{0}")]
    Noise(#[from] NoiseErrors),
    #[error("{0}")]
    Quaternion(#[from] QuaternionErrors),
    #[error("{0}")]
    Uncertainty(#[from] UncertaintyErrors),
}

/// Constant parameters for the simple star tracker sensor
/// delay - a constant in seconds between truth dynamics and the sensor measurement
/// noise_(x,y,z) - noise in arcseconds for each axis
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct StarTrackerParametersBuilder {
    delay: Option<UncertainValue>,
    misalignment: Option<UnitQuaternionBuilder>,
    noise: Option<QuaternioNoiseBuilder>,
}

impl Uncertainty for StarTrackerParametersBuilder {
    type Error = StarTrackerErrors;
    type Output = StarTrackerParameters;
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        let delay = match &self.delay {
            Some(delay) => Some(DelayedQuaternion::new(delay.sample(nominal, rng))),
            None => None,
        };
        let misalignment = match &self.misalignment {
            Some(misalignment) => Some(misalignment.sample(nominal, rng)?),
            None => None,
        };
        let noise = match &self.noise {
            Some(noise) => Some(noise.sample(nominal, rng)?),
            None => None,
        };
        Ok(StarTrackerParameters { delay, misalignment, noise })
    }
}

/// Constant parameters for the simple star tracker sensor
/// delay - a constant in seconds between truth dynamics and the sensor measurement
/// noise_(x,y,z) - noise in arcseconds for each axis
#[derive(Debug)]
struct StarTrackerParameters {
    delay: Option<DelayedQuaternion>,
    misalignment: Option<UnitQuaternion>,
    noise: Option<QuaternionNoise>,
}

#[derive(Debug, Default)]
pub struct StarTrackerState {
    noise: Option<UnitQuaternion>,
    pub measurement: UnitQuaternion,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StarTrackerBuilder {
    parameters: StarTrackerParametersBuilder,
}

impl StarTrackerBuilder {
    pub fn new() -> Self {
        Self {
            parameters: StarTrackerParametersBuilder::default(),
        }
    }

    pub fn with_delay(mut self, delay: f64) -> Self {
        self.parameters.delay = Some(UncertainValue::new(delay));
        self
    }

    pub fn with_misalignment(mut self, misalignment: UnitQuaternionBuilder) -> Self {
        self.parameters.misalignment = Some(misalignment);
        self
    }

    pub fn with_noise(mut self, noise: QuaternioNoiseBuilder) -> Self {
        self.parameters.noise = Some(noise);
        self
    }
}

impl Uncertainty for StarTrackerBuilder {
    type Error = StarTrackerErrors;
    type Output = StarTracker;
    fn sample(
        &self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        Ok(StarTracker {
            parameters: self.parameters.sample(nominal, rng)?,
            state: StarTrackerState::default(),
            telemetry: StarTrackerTelemetry::default(),
        })
    }
}

/// A star tracker attitude sensor with gaussian white noise & constant delay
/// The sensor frame is defined with +Z out the boresight, +X to the right, +Y is up
#[derive(Debug)]
pub struct StarTracker {
    parameters: StarTrackerParameters,
    pub state: StarTrackerState,
    telemetry: StarTrackerTelemetry,
}

impl SensorModel for StarTracker {
    fn update(&mut self, t: f64, connection: &BodyConnection) {
        let body = connection.body.borrow();
        let transform = &connection.transform;

        let body_to_st = UnitQuaternion::from(&transform.rotation);
        // Get the truth attitude
        let mut sensor_attitude = body_to_st * body.state.attitude_base;
        if let Some(delay) = &mut self.parameters.delay {
            delay.update(t, sensor_attitude);
            sensor_attitude = delay.get_delayed_reading(t);
        }

        // Apply optional misalignment
        if let Some(misalignment) = &self.parameters.misalignment {
            sensor_attitude = *misalignment * sensor_attitude;
        }

        // Apply optional noise
        if let Some(noise) = &mut self.parameters.noise {
            let quaternion_noise = noise.sample();
            sensor_attitude = quaternion_noise * sensor_attitude;
            self.state.noise = Some(quaternion_noise);
        } // else keep as None from initialization
        self.state.measurement = sensor_attitude;

        //update telemetry
        self.telemetry.q[0] = self.state.measurement.0.x;
        self.telemetry.q[1] = self.state.measurement.0.y;
        self.telemetry.q[2] = self.state.measurement.0.z;
        self.telemetry.q[3] = self.state.measurement.0.w;
        self.telemetry.valid = 1u8;
    }

    fn writer_save_fn(&self, writer: &mut StateWriter) {
        writer.float_buffer[0] = self.state.measurement.0.x;
        writer.float_buffer[1] = self.state.measurement.0.y;
        writer.float_buffer[2] = self.state.measurement.0.z;
        writer.float_buffer[3] = self.state.measurement.0.w;
        if let Some(noise) = self.state.noise {
            writer.float_buffer[4] = noise.0.x;
            writer.float_buffer[5] = noise.0.y;
            writer.float_buffer[6] = noise.0.z;
            writer.float_buffer[7] = noise.0.w;
        }
        writer.write_record().unwrap();
    }

    fn writer_headers(&self) -> &[&str] {
        if let Some(_) = self.state.noise {
            &[
                "measurement[x]",
                "measurement[y]",
                "measurement[z]",
                "measurement[w]",
                "noise[x]",
                "noise[y]",
                "noise[z]",
                "noise[w]",
            ]
        } else {
            &["measurement[x]", "measurement[y]", "measurement[z]", "measurement[w]"]
        }
    }

    fn write_buffer(&self, buffer: &mut HardwareBuffer) -> Result<(), SensorErrors> {
        buffer.write(&self.telemetry);
        Ok(())
    }
}

#[derive(Debug, Default, Copy, Clone, Pod, Zeroable)]
#[repr(C)]
pub struct StarTrackerTelemetry {
    q: [f64; 4],
    valid: u8,
    _padding: [u8; 7],
}
