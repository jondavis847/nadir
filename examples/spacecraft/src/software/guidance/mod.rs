use nadir_result::{NadirResult, ResultManager};
use nalgebra::Vector3;
use rotations::prelude::{Quaternion, RotationMatrix};
use serde::{Deserialize, Serialize};

use super::navigation::NavigationFsw;

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
enum TargetMode {
    #[default]
    Nadir,
    Sun,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct GuidanceFsw {
    parameters: Parameters,
    pub state: State,
    result_id: Option<u32>,
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
struct Parameters {
}

#[derive(Clone,Debug,Serialize,Deserialize,Default)]
pub struct State {
    pub target_attitude: Quaternion,
    pub target_rate: Vector3<f64>,
    target_mode: TargetMode,
}

impl GuidanceFsw {
    pub fn run(&mut self, nav: &NavigationFsw) {
        self.state.target_attitude = match self.state.target_mode {
            TargetMode::Nadir => {
                // create frame for x in velocity vector, z nadir, y completes
                let x = nav.od.state.velocity.normalize();
                let z = -nav.od.state.position.normalize();
                let y = z.cross(&x);
                let m = match RotationMatrix::from_cols(x,y,z) {
                        Ok(m) => m,
                        Err(_) => unimplemented!("better error handling for guidance")
                };
                Quaternion::from(&m)
            }
            _ => unimplemented!("implement other targeting modes")
        };

        self.state.target_rate = match self.state.target_mode {
            TargetMode::Nadir => {
                // for our spacecraft, orbit rate is about the -y-axis in the body frame
                let earth_rate = 2.0 * std::f64::consts::PI / 86400.0;
                Vector3::new(0.0,-earth_rate,0.0)
            }
            _ => Vector3::zeros()
        };
        
    }
}

impl NadirResult for GuidanceFsw {
    fn new_result(&mut self,results: &mut ResultManager) {
        // Define the actuator subfolder folder path
        let fsw_folder_path = results.result_path.join("software");

        // Check if the folder exists, if not, create it
        if !fsw_folder_path.exists() {
            std::fs::create_dir_all(&fsw_folder_path).expect("Failed to create fsw folder");
        }

        // Initialize writer using the updated path
        let headers = [
           "target_attitude[x]",
           "target_attitude[y]",
           "target_attitude[z]",
           "target_attitude[w]",
           "target_rate[x]",
           "target_rate[y]",
           "target_rate[z]",
           ];
        let id = results.new_writer("fsw_guid", &fsw_folder_path, &headers);
        self.result_id = Some(id);        
   }

   fn write_result(&self,results: &mut ResultManager) {
       if let Some(id) = self.result_id {
           results.write_record(id, &[
            self.state.target_attitude.x.to_string(),
            self.state.target_attitude.y.to_string(),
            self.state.target_attitude.z.to_string(),
            self.state.target_attitude.s.to_string(),
            self.state.target_rate[0].to_string(),
            self.state.target_rate[1].to_string(),
            self.state.target_rate[2].to_string(),
           ]);
       }        
   }
}

