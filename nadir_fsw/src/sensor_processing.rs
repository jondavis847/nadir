pub mod rate3;

pub trait SensorProcessing {
    type MeasurementState;
    fn process_sensor(&mut self);
    fn get_measurement(&self) -> Self::MeasurementState;
}