use nadir_result::{NadirResult, ResultManager};
use nalgebra::Vector3;
use rotations::prelude::Quaternion;
use serde::{Deserialize, Serialize};
use crate::software::sensors::{star_tracker::StarTrackerFsw, imu::RateGyroFsw};

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct AttitudeDetermination {
    pub state: State,
    parameters: Parameters,
    result_id: Option<u32>,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
enum AttitudeSource {
    MEKF,
    #[default]
    ST,
    Triad,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
enum RateSource {
    MEKF,
    #[default]
    Imu,
    Bdot,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct State {
    pub attitude: Quaternion,
    attitude_source: AttitudeSource,
    pub rates: Vector3<f64>,
    rate_source: RateSource,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {
}

impl AttitudeDetermination {
    pub fn run(&mut self, st: &StarTrackerFsw, imu: &RateGyroFsw) {
        if st.state.valid {
            self.state.attitude = st.state.q_body
        }; // else hold previous value?        
        self.state.attitude_source = AttitudeSource::ST;        

        if imu.state.valid {
            self.state.rates = imu.state.w_body
        }; // else hold previous value?        
        self.state.rate_source = RateSource::Imu;        
    }
}

impl NadirResult for AttitudeDetermination {
    fn new_result(&mut self,results: &mut ResultManager) {
         // Define the actuator subfolder folder path
         let fsw_folder_path = results.result_path.join("software");

         // Check if the folder exists, if not, create it
         if !fsw_folder_path.exists() {
             std::fs::create_dir_all(&fsw_folder_path).expect("Failed to create fsw folder");
         }
 
         // Initialize writer using the updated path
         let headers = [
            "attitude[x]",
            "attitude[y]",
            "attitude[z]",
            "attitude[w]",            
            "rates[x]",
            "rates[y]",
            "rates[z]",
            // "attitude_source",
            // "rate_source"
            ];
         let id = results.new_writer("fsw_nav_ad", &fsw_folder_path, &headers);
         self.result_id = Some(id);        
    }

    fn write_result(&self,results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            results.write_record(id, &[
                self.state.attitude.x.to_string(),
                self.state.attitude.y.to_string(),
                self.state.attitude.z.to_string(),
                self.state.attitude.s.to_string(),
                self.state.rates[0].to_string(),
                self.state.rates[1].to_string(),
                self.state.rates[2].to_string(),
                // self.state.attitude_source.to_string(),
                // self.state.rate_source.to_string(),
            ]);
        }        
    }
}
