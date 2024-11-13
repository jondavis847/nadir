use crate::{
    body::{BodyConnection, BodySim},
    result::{MultibodyResultTrait, ResultEntry},
    sensor::{
        noise::{NoiseModels, QuaternionNoise},
        simple::SimpleSensorResult,
        SensorResult, SensorTrait,
    },
};

use polars::prelude::*;
use rotations::{prelude::Quaternion, Rotation};
use serde::{Deserialize, Serialize};

// Noise sources from ChatGPT

// Photon Shot Noise:
// Due to the discrete nature of light, there is a statistical variation in the number of photons detected from a star over a given period. This random variation introduces shot noise, which affects the accuracy of the star position measurement on the sensor.

// Dark Current Noise:
// Even in the absence of light, electronic sensors can generate a small amount of current, known as dark current, due to thermal excitation. This creates noise, which can vary with temperature and must be carefully managed, especially for longer exposures.

// Readout Noise:
// When the sensor’s pixel values are read, each reading can introduce electronic noise, typically from the analog-to-digital converter (ADC) and other sensor circuitry. This noise can be minimized with high-quality ADCs but is still a factor in low-light or faint star tracking scenarios.

// Pixel Response Nonuniformity (PRNU):
// This noise arises from inconsistencies in the sensitivity of individual pixels on the sensor array. PRNU can cause slight variations in detected intensity across different pixels, potentially affecting the centroiding accuracy of star positions.

// Quantization Noise:
// This is introduced when the continuous analog signal from the sensor is converted to a digital signal. The limited resolution of the ADC results in small rounding errors, which become more significant at low signal levels.

// Thermal Noise:
// The sensor and its electronics generate thermal noise, especially in warmer operating conditions. This noise increases with temperature and can affect the clarity of the star signal, making it harder to achieve precise centroiding.

// Jitter and Motion Blur:
// Spacecraft vibration or motion can cause the image of stars on the sensor to blur or shift, leading to an inaccurate determination of the star’s position. This effect is typically mitigated by stabilizing the sensor or using image processing techniques.

// Cosmic Ray Hits:
// In space, cosmic rays can occasionally strike the sensor, creating high-energy noise events. These can manifest as bright spots or streaks on the sensor, which could be misinterpreted as stars if not properly filtered.

// Optical Aberrations:
// Optical imperfections such as lens distortion, chromatic aberration, and vignetting can distort the star image. This leads to systematic errors in centroiding that may vary depending on the star’s position in the field of view.

// Background Illumination Noise:
// Light from sources other than stars, such as scattered sunlight, earthshine, or zodiacal light, can increase the sensor’s background noise, making it more challenging to detect and accurately measure the star signal.

/// Constant parameters for the simple star tracker sensor
/// delay - a constant in seconds between truth dynamics and the sensor measurement
/// noise_(x,y,z) - noise in arcseconds for each axis
#[derive(Clone, Debug, Default, Serialize, Deserialize)]
struct StarTrackerParameters {
    delay: Option<f64>,
    misalignment: Option<Rotation>,
    noise: Option<QuaternionNoise>,
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct StarTrackerState {
    noise: Option<Quaternion>,
    pub measurement: Quaternion,
}

/// A star tracker attitude sensor with gaussian white noise & constant delay
/// The sensor frame is defined with +Z out the boresight, +X to the right, +Y is up
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct StarTracker {
    name: String,
    parameters: StarTrackerParameters,
    state: StarTrackerState,
    result: StarTrackerResult,
}

impl StarTracker {
    pub fn new(name: String) -> Self {
        Self {
            name,
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
            self.state.noise = Some(quaternion_noise);
        }
        self.state.measurement = sensor_attitude;
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct StarTrackerResult {
    measurement: Vec<Quaternion>,
    noise: Option<Vec<Quaternion>>,
}

impl StarTrackerResult {
    pub fn update(&mut self, sensor: &StarTracker) {
        self.measurement.push(sensor.state.measurement);
        if let Some(noise) = &sensor.state.noise {
            if let Some(noise_result) = &mut self.noise {
                noise_result.push(*noise);
            } else {
                // result was created with default as None, instantiate with Some. This will happen on first pass
                self.noise = Some(vec![*noise]);
            }
        }
    }
}

impl MultibodyResultTrait for StarTrackerResult {
    fn add_to_dataframe(&self, df: &mut DataFrame) {
        let measurement_x = Series::new(
            "measurement_x",
            self.measurement.iter().map(|q| q.x).collect::<Vec<_>>(),
        );

        let measurement_y = Series::new(
            "measurement_y",
            self.measurement.iter().map(|q| q.y).collect::<Vec<_>>(),
        );

        let measurement_z = Series::new(
            "measurement_z",
            self.measurement.iter().map(|q| q.z).collect::<Vec<_>>(),
        );

        let measurement_w = Series::new(
            "measurement_w",
            self.measurement.iter().map(|q| q.s).collect::<Vec<_>>(),
        );

        df.with_column(measurement_x).unwrap();
        df.with_column(measurement_y).unwrap();
        df.with_column(measurement_z).unwrap();
        df.with_column(measurement_w).unwrap();

        if let Some(noise) = &self.noise {
            let noise_x = Series::new("noise_x", noise.iter().map(|q| q.x).collect::<Vec<_>>());
            let noise_y = Series::new("noise_y", noise.iter().map(|q| q.y).collect::<Vec<_>>());
            let noise_z = Series::new("noise_z", noise.iter().map(|q| q.z).collect::<Vec<_>>());
            let noise_w = Series::new("noise_w", noise.iter().map(|q| q.s).collect::<Vec<_>>());

            df.with_column(noise_x).unwrap();
            df.with_column(noise_y).unwrap();
            df.with_column(noise_z).unwrap();
            df.with_column(noise_w).unwrap();
        }
    }

    fn get_state_names(&self) -> Vec<String> {
        vec!["measurement".to_string(), "noise".to_string()]
    }

    fn get_result_entry(&self) -> ResultEntry {
        ResultEntry::Sensor(SensorResult::Simple(SimpleSensorResult::StarTracker(
            self.clone(),
        )))
    }
}
