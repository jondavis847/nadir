use rotations::prelude::Quaternion;
use serde::{Deserialize, Serialize};
use crate::software::sensors::star_tracker::StarTrackerFsw;

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct AttitudeDetermination {
    state: State,
    parameters: Parameters,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
enum AttitudeSource {
    MEKF,
    #[default]
    ST,
    Triad
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct State {
    pub attitude: Quaternion,
    source: AttitudeSource,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {
}

impl AttitudeDetermination {
    pub fn run(&mut self, st: &StarTrackerFsw) {
        self.state.attitude = if st.state.valid {
            st.state.q_body
        } else {
            Quaternion::IDENTITY
        };
        self.state.source = AttitudeSource::ST;        
    }
}
