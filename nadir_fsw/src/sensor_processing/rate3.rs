use nalgebra::Vector3;
use multibody::sensor::simple::rate3::Rate3SensorState;
use transforms::Transform;
// fsw processing for the Rate3Sensor found in multibody/sensors/simple/rate3.rs

pub enum Rate3FswErrors {    
}

struct Rate3FswParameters {
    transform: Transform,
}

struct Rate3FswState {
    measurement_sensor: Vector3<f64>,
    measurement_body: Vector3<f64>,
}

pub struct Rate3Fsw {
    parameters: Rate3FswParameters,
    state: Rate3FswState,    
}

impl Rate3Fsw {
    pub fn run(&mut self, sensor_telemetry: Rate3SensorState) -> Result<(),Rate3FswErrors> {
        self.state.measurement_sensor = sensor_telemetry.measurement;
        self.state.measurement_body = self.parameters.transform * sensor_telemetry.measurement;
        Ok(())
    }
}


