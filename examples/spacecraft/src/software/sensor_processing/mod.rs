use serde::{Deserialize, Serialize};
use star_tracker::StarTrackerFsw;

use crate::hardware::SpacecraftSensors;

mod star_tracker;

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct SensorProcessing {
    st: StarTrackerFsw    
}

impl SensorProcessing {
    pub fn run(&mut self, sensors: &SpacecraftSensors) {        
        //self.imu.run(sensors.imu.model);
        self.st.run(&sensors.st.model);
    }
}