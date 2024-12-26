use crate::hardware::sensors::rate_gyro::RateGyro;
use nalgebra::Vector3;
use rotations::{prelude::Quaternion, RotationTrait};
use serde::{Deserialize,Serialize};

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct RateGyroFsw {
    state: State,
    parameters: Parameters,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct State {
    w_imu: Vector3<f64>,
    w_body: Vector3<f64>,
    valid: bool,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {
    imu_to_body: Quaternion,
}

impl RateGyroFsw {
    pub fn run(&mut self, imu: &RateGyro) {
        self.state.w_imu = imu.state.measurement;
        self.state.w_body = self.parameters.imu_to_body.transform(imu.state.measurement);
        self.state.valid = true;
    }
}