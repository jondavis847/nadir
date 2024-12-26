use crate::hardware::sensors::gps::Gps;
use nalgebra::Vector3;
use serde::{Deserialize,Serialize};

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct GpsFsw {
    state: State,
    parameters: Parameters,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct State {
    position: Vector3<f64>,
    velocity: Vector3<f64>,
    valid: bool,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {    
}

impl GpsFsw {
    pub fn run(&mut self, gps: &Gps) {
        self.state.position = gps.state.position;
        self.state.velocity = gps.state.velocity;        
        self.state.valid = true;
    }
}