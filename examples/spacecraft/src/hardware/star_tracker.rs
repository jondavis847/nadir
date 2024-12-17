use std::{fs::File, io::BufWriter};

use csv::Writer;
use multibody::{
    body::BodyConnection,
    sensor::{
        noise::{NoiseModels, QuaternionNoise},
        SensorModel,
    },
    MultibodyResult,
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

impl MultibodyResult for StarTracker {
    fn initialize_result(&self, writer: &mut Writer<BufWriter<File>>) {
        writer
            .write_record(&[
                "measurement[x]",
                "measurement[y]",
                "measurement[z]",
                "measurement[w]",
                "noise[x]",
                "noise[y]",
                "noise[z]",
                "noise[w]",
            ])
            .expect("Failed to write header");
    }

    fn write_result_file(&self, writer: &mut Writer<BufWriter<File>>) {
        writer
            .write_record(&[
                self.state.measurement.x.to_string(),
                self.state.measurement.y.to_string(),
                self.state.measurement.z.to_string(),
                self.state.measurement.s.to_string(),
                self.state.noise.x.to_string(),
                self.state.noise.y.to_string(),
                self.state.noise.z.to_string(),
                self.state.noise.s.to_string(),
            ])
            .expect("could not write star tracker result file");
    }
}
