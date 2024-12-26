use crate::hardware::sensors::star_tracker::StarTracker;
use rotations::prelude::Quaternion;
use serde::{Deserialize,Serialize};

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct StarTrackerFsw {
    state: State,
    parameters: Parameters,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct State {
    q_st: Quaternion,
    q_body: Quaternion,
    valid: bool,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {
    st_to_body: Quaternion,
}

impl StarTrackerFsw {
    pub fn run(&mut self, st: &StarTracker) {
        self.state.q_st = st.state.measurement;
        self.state.q_body = self.parameters.st_to_body * st.state.measurement;
        self.state.valid = true;
    }
}