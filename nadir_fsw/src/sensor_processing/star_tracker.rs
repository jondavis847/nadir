use multibody::sensor::simple::star_tracker::StarTrackerState;
use transforms::{prelude::Quaternion, Transform};
// fsw processing for the StarTracker found in multibody/sensors/simple/star_tracker.rs

pub enum StarTrackerFswErrors {    
}

struct StarTrackerFswParameters {
    transform: Transform,
}

struct StarTrackerFswState {
    measurement_sensor: Quaternion,
    measurement_body: Quaternion,
}

pub struct StarTrackerFsw {
    parameters: StarTrackerFswParameters,
    state: StarTrackerFswState,    
}

impl StarTrackerFsw {
    pub fn run(&mut self, sensor_telemetry: StarTrackerState) -> Result<(),StarTrackerFswErrors> {
        self.state.measurement_sensor = sensor_telemetry.measurement;
        self.state.measurement_body = Quaternion::from(&self.parameters.transform.rotation) * sensor_telemetry.measurement;
        Ok(())
    }
}


