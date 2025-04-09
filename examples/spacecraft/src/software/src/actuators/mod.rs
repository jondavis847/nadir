use nadir_result::{NadirResult, ResultManager};
use serde::{Deserialize, Serialize};
pub mod reaction_wheel;
use reaction_wheel::ReactionWheelFsw;

use crate::hardware::actuators::SpacecraftActuators;

use super::control::ControlFsw;

#[derive(Clone, Debug, Serialize, Deserialize, Default)]
pub struct ActuatorFsw {
    rw: ReactionWheelFsw,
}

impl ActuatorFsw {
    pub fn run(&mut self, control: &ControlFsw, actuators: &mut SpacecraftActuators) {
        self.rw.run(control, &mut actuators.rw);
    }

    pub fn write_results(&self, results: &mut ResultManager) {
        self.rw.write_result(results);
    }

    pub fn initialize_results(&mut self, results: &mut ResultManager) {
        self.rw.new_result(results);
    }
}
