use multibody::actuator::Actuator;
use nadir_result::{NadirResult, ResultManager};
use nalgebra::{Matrix4x3, Vector4};
use serde::{Deserialize, Serialize};

use crate::{
    hardware::actuators::reaction_wheel::{ReactionWheel, ReactionWheelCommands},
    software::control::ControlFsw,
};

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct ReactionWheelFsw {
    state: State,
    parameters: Parameters,
    result_id: Option<u32>,
}

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
struct State {
    wheel_torque_commands: Vector4<f64>,
    wheel_current_commands: Vector4<f64>,
}

#[derive(Clone, Debug, Serialize, Deserialize)]
struct Parameters {
    kt: [f64; 4],
    body_to_wheel: Matrix4x3<f64>,
    max_torque: f64,
}

impl Default for Parameters {
    fn default() -> Self {
        Parameters {
            max_torque: 0.5,
            kt: [0.5, 0.5, 0.5, 0.5],
            body_to_wheel: Matrix4x3::new(
                1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
            ),
        }
    }
}

impl ReactionWheelFsw {
    pub fn run(&mut self, control: &ControlFsw, rw: &mut [Actuator<ReactionWheel>; 4]) {
        // wheel torque commands need to be equal and oppositie of body torque commmands, so negative sign

        let wheel_torque_commands =
            self.parameters.body_to_wheel * (-control.state.torque_cmd_body);

        // Find the maximum absolute wheel torque
        let max_torque = {
            let mut max_torque = 0.0;
            for i in 0..4 {
                if wheel_torque_commands[i].abs() > max_torque {
                    max_torque = wheel_torque_commands[i].abs();
                }
            }
            max_torque
        };

        // Determine if scaling is needed
        let scaling_factor = if max_torque > self.parameters.max_torque {
            self.parameters.max_torque / max_torque
        } else {
            1.0
        };

        for i in 0..4 {
            self.state.wheel_torque_commands[i] = scaling_factor * wheel_torque_commands[i];
            self.state.wheel_current_commands[i] =
                self.state.wheel_torque_commands[i] / self.parameters.kt[i];
        }

        for (i, wheel) in rw.iter_mut().enumerate() {
            wheel.model.state.command =
                ReactionWheelCommands::Torque(self.state.wheel_torque_commands[i]);
        }
    }
}

impl NadirResult for ReactionWheelFsw {
    fn new_result(&mut self, results: &mut ResultManager) {
        // Define the actuator subfolder folder path
        let fsw_folder_path = results.result_path.join("software");

        // Check if the folder exists, if not, create it
        if !fsw_folder_path.exists() {
            std::fs::create_dir_all(&fsw_folder_path).expect("Failed to create fsw folder");
        }

        // Initialize writer using the updated path
        let headers = [
            "wheel_torque_commands[0]",
            "wheel_torque_commands[1]",
            "wheel_torque_commands[2]",
            "wheel_torque_commands[3]",
            "wheel_current_commands[0]",
            "wheel_current_commands[1]",
            "wheel_current_commands[2]",
            "wheel_current_commands[3]",
        ];
        let id = results.new_writer("fsw_rw", &fsw_folder_path, &headers);
        self.result_id = Some(id);
    }

    fn write_result(&self, results: &mut ResultManager) {
        if let Some(id) = self.result_id {
            results.write_record(
                id,
                &[
                    self.state.wheel_torque_commands[0].to_string(),
                    self.state.wheel_torque_commands[1].to_string(),
                    self.state.wheel_torque_commands[2].to_string(),
                    self.state.wheel_torque_commands[3].to_string(),
                    self.state.wheel_current_commands[0].to_string(),
                    self.state.wheel_current_commands[1].to_string(),
                    self.state.wheel_current_commands[2].to_string(),
                    self.state.wheel_current_commands[3].to_string(),
                ],
            );
        }
    }
}
