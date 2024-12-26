use nalgebra::Vector3;
use serde::{Deserialize, Serialize};

use super::sensors::SensorFsw;


#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct GuidanceFsw {
    parameters: Parameters,
    state: State,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {
    // Control gains
    k_p: f64,
    k_i: f64,
    k_d: f64,    
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct State {
    error: Vector3<f64>,
    integral: Vector3<f64>,
    derivative: Vector3<f64>,
}

impl GuidanceFsw {
    pub fn run(&mut self, sensors: &SensorFsw) {
        let error = sensors.st.error;
        self.state.error = error;
        self.state.integral += error;
        self.state.derivative = error - self.state.error;
        let control = self.parameters.k_p * error + self.parameters.k_i * self.state.integral + self.parameters.k_d * self.state.derivative;
        // send control to actuators
    }
}