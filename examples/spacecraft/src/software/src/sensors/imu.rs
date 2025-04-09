use bytemuck::{Pod, Zeroable};
use multibody::HardwareBuffer;
use nadir_result::{NadirResult, ResultManager};
use nalgebra::Vector3;
use rotations::{prelude::UnitQuaternion, RotationTrait};

#[derive(Clone, Debug, Default)]
pub struct RateGyroFsw {
    pub state: State,
    parameters: Parameters,
    result_id: Option<u32>,
    telemetry: RateGyroTelemetry,
}

#[derive(Clone, Debug, Default)]
pub struct State {
    w_imu: Vector3<f64>,
    pub w_body: Vector3<f64>,
    pub valid: bool,
}

#[derive(Debug, Default, Pod, Zeroable, Clone, Copy)]
#[repr(C)]
pub struct RateGyroTelemetry {
    w: [f64; 3],
    valid: u8,
    _padding: [u8; 7],
}

#[derive(Clone, Debug, Default)]
struct Parameters {
    imu_to_body: UnitQuaternion,
}

impl RateGyroFsw {
    pub fn run(&mut self) {
        self.state.w_imu = self.telemetry.w.into();
        self.state.w_body = self.parameters.imu_to_body.transform(&self.state.w_imu);
        self.state.valid = match self.telemetry.valid {
            0u8 => false,
            1u8 => true,
            _ => panic!("invalid rate gyro validity flag"),
        };
    }

    pub fn read_buffer(&mut self, buffer: &HardwareBuffer) {
        if let Some(telemetry) = buffer.read::<RateGyroTelemetry>() {
            self.telemetry.clone_from(&telemetry);
        }
    }
}

impl NadirResult for RateGyroFsw {
    fn new_result(&mut self, results: &mut ResultManager) {
        // Define the actuator subfolder folder path
        let fsw_folder_path = results.result_path.join("software");

        // Check if the folder exists, if not, create it
        if !fsw_folder_path.exists() {
            std::fs::create_dir_all(&fsw_folder_path).expect("Failed to create fsw folder");
        }

        // Initialize writer using the updated path
        let headers = [
            "w_imu[x]",
            "w_imu[y]",
            "w_imu[z]",
            "w_body[x]",
            "w_body[y]",
            "w_body[z]",
            "valid",
        ];
        let id = results.new_writer("fsw_imu", &fsw_folder_path, &headers);
        self.result_id = Some(id);
    }

    fn write_result(&self, results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            results.write_record(
                id,
                &[
                    self.state.w_imu[0].to_string(),
                    self.state.w_imu[1].to_string(),
                    self.state.w_imu[2].to_string(),
                    self.state.w_body[0].to_string(),
                    self.state.w_body[1].to_string(),
                    self.state.w_body[2].to_string(),
                    self.state.valid.to_string(),
                ],
            );
        }
    }
}
