use bytemuck::{Pod, Zeroable};
use multibody::HardwareBuffer;
use nadir_result::{NadirResult, ResultManager};
use rotations::prelude::UnitQuaternion;

#[derive(Debug, Default)]
pub struct StarTrackerFsw {
    pub state: State,
    parameters: Parameters,
    telemetry: StarTrackerTelemetry,
    result_id: Option<u32>,
}

#[derive(Debug, Default)]
pub struct State {
    pub q_st: UnitQuaternion,
    pub q_body: UnitQuaternion,
    pub valid: bool,
}

#[derive(Debug, Default)]
struct Parameters {
    st_to_body: UnitQuaternion,
}

#[derive(Debug, Default, Pod, Zeroable, Clone, Copy)]
#[repr(C)]
pub struct StarTrackerTelemetry {
    q_st: [f64; 4],
    valid: u8,
    _padding: [u8; 7],
}

impl StarTrackerFsw {
    pub fn run(&mut self) {
        self.state.q_st = UnitQuaternion::new(
            self.telemetry.q_st[0],
            self.telemetry.q_st[1],
            self.telemetry.q_st[2],
            self.telemetry.q_st[3],
        );
        self.state.q_body = self.parameters.st_to_body * self.state.q_st;
        self.state.valid = match self.telemetry.valid {
            0u8 => false,
            1u8 => true,
            _ => panic!("invalid star tracker validity flag"),
        };
    }

    pub fn read_buffer(&mut self, buffer: &HardwareBuffer) {
        if let Some(telemetry) = buffer.read::<StarTrackerTelemetry>() {
            self.telemetry.clone_from(&telemetry);
        }
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
                    self.state.q_st.0.x.to_string(),
                    self.state.q_st.0.y.to_string(),
                    self.state.q_st.0.z.to_string(),
                    self.state.q_st.0.w.to_string(),
                    self.state.q_body.0.x.to_string(),
                    self.state.q_body.0.y.to_string(),
                    self.state.q_body.0.z.to_string(),
                    self.state.q_body.0.w.to_string(),
                    self.state.valid.to_string(),
                ],
            );
        }
    }
}
