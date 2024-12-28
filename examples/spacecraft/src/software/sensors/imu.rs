use crate::hardware::sensors::rate_gyro::RateGyro;
use nadir_result::{NadirResult, ResultManager};
use nalgebra::Vector3;
use rotations::{prelude::Quaternion, RotationTrait};
use serde::{Deserialize,Serialize};

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct RateGyroFsw {
    pub state: State,
    parameters: Parameters,
    result_id: Option<u32>,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct State {
    w_imu: Vector3<f64>,
    pub w_body: Vector3<f64>,
    pub valid: bool,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {
    imu_to_body: Quaternion,
}

impl RateGyroFsw {
    pub fn run(&mut self, imu: &RateGyro) {
        self.state.w_imu = imu.state.measurement;
        self.state.w_body = self.parameters.imu_to_body.transform(imu.state.measurement);
        self.state.valid = true;
    }
}

impl NadirResult for RateGyroFsw {
    fn new_result(&mut self,results: &mut ResultManager) {
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

    fn write_result(&self,results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            results.write_record(id, &[
                self.state.w_imu[0].to_string(),
                self.state.w_imu[1].to_string(),
                self.state.w_imu[2].to_string(),
                self.state.w_body[0].to_string(),
                self.state.w_body[1].to_string(),
                self.state.w_body[2].to_string(),
                self.state.valid.to_string(),            
            ]);
        }        
    }
}