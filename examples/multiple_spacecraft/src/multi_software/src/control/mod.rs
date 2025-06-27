use nadir_result::{NadirResult, ResultManager};
use nalgebra::Vector3;
use rotations::RotationTrait;

use super::{guidance::GuidanceFsw, navigation::NavigationFsw};

#[derive(Debug, Default)]
pub struct ControlFsw {
    parameters: Parameters,
    pub state: State,
    result_id: Option<u32>,
}

#[derive(Debug)]
struct Parameters {
    // Control gains
    k_p: f64,
    k_i: f64,
    k_d: f64,
    anti_windup: f64,
    moi: [f64; 3],
}

impl Default for Parameters {
    fn default() -> Self {
        Parameters {
            k_p: 0.5,
            k_i: 0.000005, //we account for dt in the gain, since i might change dt when testing in sim
            k_d: 5.0,
            anti_windup: 0.0075,
            moi: [10.0, 10.0, 10.0],
        }
    }
}

#[derive(Debug, Default)]
pub struct State {
    attitude_error: Vector3<f64>,
    rate_error: Vector3<f64>,
    integral_error: Vector3<f64>,
    pub torque_cmd_body: Vector3<f64>,
}

impl ControlFsw {
    pub fn run(&mut self, nav: &NavigationFsw, guid: &GuidanceFsw) {
        // Ensure quaternions are normalized
        let current_attitude = nav
            .ad
            .state
            .attitude;
        let target_attitude = guid
            .state
            .target_attitude;

        // Compute the error quaternion: q_error = q_current.inv() * q_target
        let q_error = current_attitude * target_attitude.inv();

        // Ensure the scalar part is non-negative to represent the shortest rotation
        let q_error = if q_error
            .0
            .w
            < 0.0
        {
            -q_error
        } else {
            q_error
        };

        // Compute the attitude error vector (scaled by 2 for small angles)
        self.state
            .attitude_error = Vector3::new(
            q_error
                .0
                .x,
            q_error
                .0
                .y,
            q_error
                .0
                .z,
        );

        // Rate Error
        self.state
            .rate_error = nav
            .ad
            .state
            .rates
            - guid
                .state
                .target_rate;

        // Integral Error
        for i in 0..3 {
            if self
                .state
                .attitude_error[i]
                .abs()
                < self
                    .parameters
                    .anti_windup
            {
                self.state
                    .integral_error[i] += self
                    .state
                    .attitude_error[i];
            } else {
                self.state
                    .integral_error[i] = 0.0;
            }
        }

        // Torque Command
        for i in 0..3 {
            self.state
                .torque_cmd_body[i] = self
                .parameters
                .moi[i]
                * (-self
                    .parameters
                    .k_p
                    * self
                        .state
                        .attitude_error[i]
                    - self
                        .parameters
                        .k_i
                        * self
                            .state
                            .integral_error[i]
                    - self
                        .parameters
                        .k_d
                        * self
                            .state
                            .rate_error[i]);
        }
    }
}

impl NadirResult for ControlFsw {
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
            "attitude_error[x]",
            "attitude_error[y]",
            "attitude_error[z]",
            "rate_error[x]",
            "rate_error[y]",
            "rate_error[z]",
            "integral_error[x]",
            "integral_error[y]",
            "integral_error[z]",
            "torque_cmd_body[x]",
            "torque_cmd_body[y]",
            "torque_cmd_body[z]",
        ];
        let id = results.new_writer(
            "fsw_ctrl",
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
                        .attitude_error[0]
                        .to_string(),
                    self.state
                        .attitude_error[1]
                        .to_string(),
                    self.state
                        .attitude_error[2]
                        .to_string(),
                    self.state
                        .rate_error[0]
                        .to_string(),
                    self.state
                        .rate_error[1]
                        .to_string(),
                    self.state
                        .rate_error[2]
                        .to_string(),
                    self.state
                        .integral_error[0]
                        .to_string(),
                    self.state
                        .integral_error[1]
                        .to_string(),
                    self.state
                        .integral_error[2]
                        .to_string(),
                    self.state
                        .torque_cmd_body[0]
                        .to_string(),
                    self.state
                        .torque_cmd_body[1]
                        .to_string(),
                    self.state
                        .torque_cmd_body[2]
                        .to_string(),
                ],
            );
        }
    }
}
