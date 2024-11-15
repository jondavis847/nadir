use std::collections::HashMap;

use crate::{
    body::{BodyConnection, BodySim},
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{
        noise::{NoiseModels, QuaternionNoise},
        simple::SimpleSensorResult,
        SensorResult, SensorTrait,
    }, MultibodyErrors,
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

    pub fn with_delay(mut self, delay: f64) -> Self {
        self.parameters.delay = Some(delay);
        self
    }

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

impl SensorTrait for StarTracker {
    fn initialize_result(&self) -> SensorResult {
        SensorResult::Simple(SimpleSensorResult::StarTracker(StarTrackerResult::default()))
    }

    fn update(&mut self, body: &BodySim, connection: &BodyConnection) {
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

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StarTrackerResult(HashMap<String, Vec<f64>>);

impl StarTrackerResult {
    pub fn new() -> Self {
        let mut result = HashMap::new();
        Self::STATES.iter().for_each(|state| {
            result.insert(state.to_string(), Vec::new());
        });
        Self(result)
    }
}

impl MultibodyResultTrait for StarTrackerResult {
    type Component = StarTracker;
    const STATES: &'static [&'static str] = &[
        "measurement[x]",
        "measurement[y]",
        "measurement[z]",
        "measurement[w]",
        "noise[x]",
        "noise[y]",
        "noise[z]",
        "noise[w]",
    ];
    fn get_state_value(&self, state: &str) -> Result<&Vec<f64>, MultibodyErrors> {
        self.0.get(state).ok_or(MultibodyErrors::ComponentStateNotFound(state.to_string()))
    }

    fn get_state_names(&self) -> &'static [&'static str] {
        Self::STATES
    }
    
    fn get_result_entry(&self) -> ResultEntry {
        ResultEntry::Sensor(SensorResult::Simple(SimpleSensorResult::StarTracker(
            self.clone(),
        )))
    }

    fn update(&mut self, sensor: &StarTracker) {
        self.0
            .get_mut("measurement[x]")
            .unwrap()
            .push(sensor.state.measurement.x);
        self.0
            .get_mut("measurement[y]")
            .unwrap()
            .push(sensor.state.measurement.y);
        self.0
            .get_mut("measurement[z]")
            .unwrap()
            .push(sensor.state.measurement.z);
        self.0
            .get_mut("measurement[w]")
            .unwrap()
            .push(sensor.state.measurement.s);
        self.0
            .get_mut("noise[x]")
            .unwrap()
            .push(sensor.state.noise.x);
        self.0
            .get_mut("noise[y]")
            .unwrap()
            .push(sensor.state.noise.y);
        self.0
            .get_mut("noise[z]")
            .unwrap()
            .push(sensor.state.noise.z);
        self.0
            .get_mut("noise[w]")
            .unwrap()
            .push(sensor.state.noise.s);
    }
}
