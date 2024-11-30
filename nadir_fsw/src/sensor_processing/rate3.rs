use multibody::sensor::examples::rate3::Rate3SensorState;
use nalgebra::Vector3;
use serde::{Deserialize, Serialize};
use thiserror::Error;
use transforms::{prelude::RotationTrait, Transform};
// fsw processing for the Rate3Sensor found in multibody/sensors/simple/rate3.rs

#[derive(Debug, Error)]
pub enum Rate3FswErrors {}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
struct Rate3FswParameters {
    transform: Transform,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
struct Rate3FswState {
    measurement_sensor: Vector3<f64>,
    measurement_body: Vector3<f64>,
}

#[derive(Copy, Clone, Debug, Serialize, Deserialize)]
pub struct Rate3Fsw {
    parameters: Rate3FswParameters,
    state: Rate3FswState,
}

impl Rate3Fsw {
    pub fn run(&mut self, sensor_telemetry: Rate3SensorState) -> Result<(), Rate3FswErrors> {
        self.state.measurement_sensor = sensor_telemetry.measurement;
        self.state.measurement_body = self.parameters.transform.rotation.transform(sensor_telemetry.measurement);
        Ok(())
    }
}
