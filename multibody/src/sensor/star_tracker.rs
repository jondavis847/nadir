use super::noise::{NoiseErrors, QuaternioNoiseBuilder};
use crate::{
    body::BodyConnection,
    sensor::{noise::QuaternionNoise, SensorModel},
};
use rotations::prelude::{QuaternionErrors, UnitQuaternion, UnitQuaternionBuilder};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use uncertainty::{SimValue, Uncertainty, UncertaintyErrors};

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
    delay: Option<SimValue>,
    misalignment: Option<UnitQuaternionBuilder>,
    noise: Option<QuaternioNoiseBuilder>,
}

impl Uncertainty for StarTrackerParametersBuilder {
    type Error = StarTrackerErrors;
    type Output = StarTrackerParameters;
    fn sample(
        &mut self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        let delay = match &mut self.delay {
            Some(delay) => Some(delay.sample(nominal, rng)),
            None => None,
        };
        let misalignment = match &mut self.misalignment {
            Some(misalignment) => Some(misalignment.sample(nominal, rng)?),
            None => None,
        };
        let noise = match &mut self.noise {
            Some(noise) => Some(noise.sample(nominal, rng)?),
            None => None,
        };
        Ok(StarTrackerParameters {
            delay,
            misalignment,
            noise,
        })
    }
}

/// Constant parameters for the simple star tracker sensor
/// delay - a constant in seconds between truth dynamics and the sensor measurement
/// noise_(x,y,z) - noise in arcseconds for each axis
#[derive(Debug)]
struct StarTrackerParameters {
    delay: Option<f64>,
    misalignment: Option<UnitQuaternion>,
    noise: Option<QuaternionNoise>,
}

#[derive(Debug, Default)]
pub struct StarTrackerState {
    noise: UnitQuaternion,
    pub measurement: UnitQuaternion,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StarTrackerBuilder {
    parameters: StarTrackerParametersBuilder,
}

impl Uncertainty for StarTrackerBuilder {
    type Error = StarTrackerErrors;
    type Output = StarTracker;
    fn sample(
        &mut self,
        nominal: bool,
        rng: &mut rand::prelude::SmallRng,
    ) -> Result<Self::Output, Self::Error> {
        Ok(StarTracker {
            parameters: self.parameters.sample(nominal, rng)?,
            state: StarTrackerState::default(),
        })
    }
}

/// A star tracker attitude sensor with gaussian white noise & constant delay
/// The sensor frame is defined with +Z out the boresight, +X to the right, +Y is up
#[derive(Debug)]
pub struct StarTracker {
    parameters: StarTrackerParameters,
    pub state: StarTrackerState,
}

impl SensorModel for StarTracker {
    fn update(&mut self, connection: &BodyConnection) {
        let body = connection.body.borrow();
        let transform = &connection.transform;

        let body_to_st = UnitQuaternion::from(&transform.rotation);
        // Get the truth attitude
        let mut sensor_attitude = body_to_st * body.state.attitude_base;
        // Apply optional misalignment
        if let Some(misalignment) = &self.parameters.misalignment {
            sensor_attitude = *misalignment * sensor_attitude;
        }

        // Apply optional noise
        if let Some(noise) = &mut self.parameters.noise {
            let quaternion_noise = noise.sample();
            sensor_attitude = quaternion_noise * sensor_attitude;
            self.state.noise = quaternion_noise;
        } // else keep as identity from initialization
        self.state.measurement = sensor_attitude;
    }

    fn result_content(&self, id: u32, results: &mut nadir_result::ResultManager) {
        let content = &[
            self.state.measurement.0.x.to_string(),
            self.state.measurement.0.y.to_string(),
            self.state.measurement.0.z.to_string(),
            self.state.measurement.0.w.to_string(),
            self.state.noise.0.x.to_string(),
            self.state.noise.0.y.to_string(),
            self.state.noise.0.z.to_string(),
            self.state.noise.0.w.to_string(),
        ];
        results.write_record(id, content);
    }

    fn result_headers(&self) -> &[&str] {
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
    }
}
