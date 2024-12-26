use crate::hardware::sensors::gps::Gps;
use nalgebra::Vector3;
use serde::{Deserialize,Serialize};
use time::Time;

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct GpsFsw {
    pub state: State,
    parameters: Parameters,
}

#[derive(Clone,Debug,Serialize,Deserialize)]
struct State {
    pub time: Time,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub valid: bool,
}

impl Default for State {
    fn default() -> Self {
        State {
            time: Time::from_sec_j2k(0.0,time::TimeSystem::TAI),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(), 
            valid: false,
        }
    }    
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {    
}

impl GpsFsw {
    pub fn run(&mut self, gps: &Gps) {
        self.state.time = gps.state.time;
        self.state.position = gps.state.position;
        self.state.velocity = gps.state.velocity;        
        self.state.valid = true;
    }
}