pub mod rate3;
pub mod star_tracker;

pub trait SensorProcessing {
    type MeasurementState;
    fn process_sensor(&mut self);
    fn get_measurement(&self) -> Self::MeasurementState;
}