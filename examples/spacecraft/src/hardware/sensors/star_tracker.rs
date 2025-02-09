use multibody::{
    body::BodyConnection,
    sensor::{
        noise::{NoiseModels, QuaternionNoise},
        SensorModel,
    },
};

use rotations::{prelude::UnitQuaternion, Rotation};
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
    noise: UnitQuaternion,
    pub measurement: UnitQuaternion,
}

/// A star tracker attitude sensor with gaussian white noise & constant delay
/// The sensor frame is defined with +Z out the boresight, +X to the right, +Y is up
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StarTracker {
    parameters: StarTrackerParameters,
    pub state: StarTrackerState,
}

impl StarTracker {
    pub fn new() -> Self {
        Self {
            parameters: StarTrackerParameters::default(),
            state: StarTrackerState::default(),
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

impl SensorModel for StarTracker {
    fn update(&mut self, connection: &BodyConnection) {
        let body = connection.body.borrow();
        let body_to_st = UnitQuaternion::from(&connection.transform.rotation);
        // Get the truth attitude
        let mut sensor_attitude = body_to_st * body.state.attitude_base;
        // Apply optional misalignment
        if let Some(misalignment) = &self.parameters.misalignment {
            sensor_attitude = UnitQuaternion::from(misalignment) * sensor_attitude;
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
