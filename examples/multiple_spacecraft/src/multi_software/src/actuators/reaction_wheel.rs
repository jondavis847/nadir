use crate::control::ControlFsw;
use bytemuck::{Pod, Zeroable};
use multibody::HardwareBuffer;
use nadir_result::{NadirResult, ResultManager};
use nalgebra::Matrix3;

#[derive(Debug, Default, Pod, Zeroable, Copy, Clone)]
#[repr(C)]
pub struct ReactionWheelCommand {
    value: f64,
    command: u8,
    _padding: [u8; 7],
}

impl ReactionWheelCommand {
    const TORQUE: u8 = 0;
    const CURRENT: u8 = 1;
    const SPEED: u8 = 2;
}

#[derive(Debug, Default)]
pub struct State {
    pub command: [ReactionWheelCommand; 3],
}

#[derive(Debug)]
struct Parameters {
    kt: [f64; 3],
    body_to_wheel: Matrix3<f64>,
    max_torque: f64,
}

impl Default for Parameters {
    fn default() -> Self {
        Parameters {
            max_torque: 0.5,
            kt: [0.5, 0.5, 0.5],
            body_to_wheel: Matrix3::new(
                1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0,
            ),
        }
    }
}

#[derive(Debug, Default)]
pub struct ReactionWheelFsw {
    pub state: State,
    parameters: Parameters,
    result_id: Option<u32>,
}

impl ReactionWheelFsw {
    pub fn run(&mut self, control: &ControlFsw) {
        // wheel torque commands need to be equal and oppositie of body torque commmands, so negative sign

        let wheel_torque_commands = self
            .parameters
            .body_to_wheel
            * (-control
                .state
                .torque_cmd_body);

        // Find the maximum absolute wheel torque
        let max_torque = {
            let mut max_torque = 0.0;
            for i in 0..3 {
                if wheel_torque_commands[i].abs() > max_torque {
                    max_torque = wheel_torque_commands[i].abs();
                }
            }
            max_torque
        };

        // Determine if scaling is needed to maintain body torque direction
        let scaling_factor = if max_torque
            > self
                .parameters
                .max_torque
        {
            self.parameters
                .max_torque
                / max_torque
        } else {
            1.0
        };

        for i in 0..3 {
            self.state
                .command[i] = ReactionWheelCommand {
                command: ReactionWheelCommand::TORQUE,
                value: scaling_factor * wheel_torque_commands[i],
                _padding: [0; 7],
            };
        }
    }

    pub fn write_buffer(&self, buffer: &mut [HardwareBuffer]) {
        for (i, rw) in self
            .state
            .command
            .iter()
            .enumerate()
        {
            buffer[i].write(rw)
        }
    }
}

impl NadirResult for ReactionWheelFsw {
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
            "wheel_command_mode[0]",
            "wheel_command_mode[1]",
            "wheel_command_mode[2]",
            "wheel_command_value[0]",
            "wheel_command_value[1]",
            "wheel_command_value[2]",
        ];
        let id = results.new_writer(
            "fsw_rw",
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
                        .command[0]
                        .command
                        .to_string(),
                    self.state
                        .command[1]
                        .command
                        .to_string(),
                    self.state
                        .command[2]
                        .command
                        .to_string(),
                    self.state
                        .command[0]
                        .value
                        .to_string(),
                    self.state
                        .command[1]
                        .value
                        .to_string(),
                    self.state
                        .command[2]
                        .value
                        .to_string(),
                ],
            );
        }
    }
}
