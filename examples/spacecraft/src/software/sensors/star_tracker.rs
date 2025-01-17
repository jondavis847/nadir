use crate::hardware::sensors::star_tracker::StarTracker;
use nadir_result::{NadirResult, ResultManager};
use rotations::prelude::UnitQuaternion;
use serde::{Deserialize, Serialize};

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct StarTrackerFsw {
    pub state: State,
    parameters: Parameters,
    result_id: Option<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct State {
    pub q_st: UnitQuaternion,
    pub q_body: UnitQuaternion,
    pub valid: bool,
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
struct Parameters {
    st_to_body: UnitQuaternion,
}

impl StarTrackerFsw {
    pub fn run(&mut self, st: &StarTracker) {
        self.state.q_st = st.state.measurement;
        self.state.q_body = self.parameters.st_to_body * st.state.measurement;
        self.state.valid = true;
    }
}

impl NadirResult for StarTrackerFsw {
    fn new_result(&mut self, results: &mut ResultManager) {
        // Define the actuator subfolder folder path
        let fsw_folder_path = results.result_path.join("software");

        // Check if the folder exists, if not, create it
        if !fsw_folder_path.exists() {
            std::fs::create_dir_all(&fsw_folder_path).expect("Failed to create fsw folder");
        }

        // Initialize writer using the updated path
        let headers = [
            "q_st[x]",
            "q_st[y]",
            "q_st[z]",
            "q_st[w]",
            "q_body[x]",
            "q_body[y]",
            "q_body[z]",
            "q_body[w]",
            "valid",
        ];
        let id = results.new_writer("fsw_st", &fsw_folder_path, &headers);
        self.result_id = Some(id);
    }

    fn write_result(&self, results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            results.write_record(
                id,
                &[
                    self.state.q_st.x.to_string(),
                    self.state.q_st.y.to_string(),
                    self.state.q_st.z.to_string(),
                    self.state.q_st.w.to_string(),
                    self.state.q_body.x.to_string(),
                    self.state.q_body.y.to_string(),
                    self.state.q_body.z.to_string(),
                    self.state.q_body.w.to_string(),
                    self.state.valid.to_string(),
                ],
            );
        }
    }
}
