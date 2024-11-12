use multibody::sensor::simple::star_tracker::StarTrackerState;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use transforms::{prelude::Quaternion, Transform};
// fsw processing for the StarTracker found in multibody/sensors/simple/star_tracker.rs

#[derive(Debug, Error)]
pub enum StarTrackerFswErrors {}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
struct StarTrackerFswParameters {
    transform: Transform,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
struct StarTrackerFswState {
    measurement_sensor: Quaternion,
    measurement_body: Quaternion,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub struct StarTrackerFsw {
    parameters: StarTrackerFswParameters,
    state: StarTrackerFswState,
}

impl StarTrackerFsw {
    pub fn run(&mut self, sensor_telemetry: StarTrackerState) -> Result<(), StarTrackerFswErrors> {
        self.state.measurement_sensor = sensor_telemetry.measurement;
        self.state.measurement_body =
            Quaternion::from(&self.parameters.transform.rotation) * sensor_telemetry.measurement;
        Ok(())
    }
}
