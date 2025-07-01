use gps::GpsFsw;
use imu::RateGyroFsw;
use multibody::HardwareBuffer;
use nadir_result::{NadirResult, ResultManager};
use star_tracker::StarTrackerFsw;

pub mod gps;
pub mod imu;
pub mod star_tracker;

#[derive(Debug, Default)]
pub struct SensorFsw {
    pub gps: GpsFsw,
    pub imu: RateGyroFsw,
    pub st: StarTrackerFsw,
}

impl SensorFsw {
    pub fn read_buffers(&mut self, sensor_data: &[HardwareBuffer]) {
        self.gps
            .read_buffer(&sensor_data[0]);
        self.st
            .read_buffer(&sensor_data[1]);
        self.imu
            .read_buffer(&sensor_data[2]);
    }
    pub fn run(&mut self) {
        self.gps
            .run();
        self.st
            .run();
        self.imu
            .run();
    }

    pub fn write_results(&self, results: &mut ResultManager) {
        self.gps
            .write_result(results);
        self.st
            .write_result(results);
        self.imu
            .write_result(results);
    }

    pub fn initialize_results(&mut self, results: &mut ResultManager) {
        self.gps
            .new_result(results);
        self.st
            .new_result(results);
        self.imu
            .new_result(results);
    }
}
