use bytemuck::{Pod, Zeroable};
use multibody::HardwareBuffer;
use nadir_result::{NadirResult, ResultManager};
use nalgebra::Vector3;
use time::Time;

#[derive(Debug, Default)]
pub struct GpsFsw {
    pub state: State,
    parameters: Parameters,
    result_id: Option<u32>,
    telemetry: GpsTelemetry,
}

#[derive(Debug)]
pub struct State {
    pub time: Time,
    pub position: Vector3<f64>,
    pub velocity: Vector3<f64>,
    pub valid: bool,
}

impl Default for State {
    fn default() -> Self {
        State {
            time: Time::from_sec_j2k(0.0, time::TimeSystem::TAI),
            position: Vector3::zeros(),
            velocity: Vector3::zeros(),
            valid: false,
        }
    }
}

#[derive(Debug, Default, Pod, Zeroable, Clone, Copy)]
#[repr(C)]
pub struct GpsTelemetry {
    time: f64,
    position: [f64; 3],
    velocity: [f64; 3],
    valid: u8,
    _padding: [u8; 7],
}

#[derive(Debug, Default)]
struct Parameters {}

impl GpsFsw {
    pub fn run(&mut self) {
        self.state
            .time = Time::from_sec_j2k(
            self.telemetry
                .time,
            time::TimeSystem::TAI,
        );
        self.state
            .position = self
            .telemetry
            .position
            .into();
        self.state
            .velocity = self
            .telemetry
            .velocity
            .into();
        self.state
            .valid = match self
            .telemetry
            .valid
        {
            0u8 => false,
            1u8 => true,
            _ => panic!("invalid gps validity flag"),
        };
    }

    pub fn read_buffer(&mut self, buffer: &HardwareBuffer) {
        match buffer.read::<GpsTelemetry>() {
            Ok(telemetry) => self
                .telemetry
                .clone_from(&telemetry),
            Err(e) => eprintln!("{e}"),
        }
    }
}

impl NadirResult for GpsFsw {
    fn new_result(&mut self, results: &mut ResultManager) {
        // Define the actuator subfolder folder path
        let fsw_folder_path = results
            .result_path
            .join("software");

        // Check if the folder exists, if not, create it
        if !fsw_folder_path.exists() {
            std::fs::create_dir_all(&fsw_folder_path).expect("Failed to create fsw folder");
        }

        // Initialize writer using the updated path
        let headers = [
            "time(jd)",
            "position(j2k)[x]",
            "position(j2k)[y]",
            "position(j2k)[z]",
            "velocity(j2k)[x]",
            "velocity(j2k)[y]",
            "velocity(j2k)[z]",
            "valid",
        ];
        let id = results.new_writer(
            "fsw_gps",
            &fsw_folder_path,
            &headers,
        );
        self.result_id = Some(id);
    }

    fn write_result(&self, results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            results.write_record(
                id,
                &[
                    self.state
                        .time
                        .get_datetime()
                        .to_string(),
                    self.state
                        .position
                        .x
                        .to_string(),
                    self.state
                        .position
                        .y
                        .to_string(),
                    self.state
                        .position
                        .z
                        .to_string(),
                    self.state
                        .velocity
                        .x
                        .to_string(),
                    self.state
                        .velocity
                        .y
                        .to_string(),
                    self.state
                        .velocity
                        .z
                        .to_string(),
                    self.state
                        .valid
                        .to_string(),
                ],
            );
        }
    }
}
