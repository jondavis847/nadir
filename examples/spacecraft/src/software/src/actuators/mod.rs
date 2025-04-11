use super::control::ControlFsw;
use multibody::HardwareBuffer;
use nadir_result::{NadirResult, ResultManager};
use reaction_wheel::ReactionWheelFsw;
pub mod reaction_wheel;

#[derive(Debug, Default)]
pub struct ActuatorFsw {
    rw: ReactionWheelFsw,
}

impl ActuatorFsw {
    pub fn run(&mut self, control: &ControlFsw) {
        self.rw.run(control);
    }

    pub fn write_buffers(&self, buffers: &mut [HardwareBuffer]) {
        buffers[0].write(&self.rw.state.command[0]);
        buffers[1].write(&self.rw.state.command[1]);
        buffers[2].write(&self.rw.state.command[2]);
        buffers[3].write(&self.rw.state.command[3]);
    }

    pub fn write_results(&self, results: &mut ResultManager) {
        self.rw.write_result(results);
    }

    pub fn initialize_results(&mut self, results: &mut ResultManager) {
        self.rw.new_result(results);
    }
}
