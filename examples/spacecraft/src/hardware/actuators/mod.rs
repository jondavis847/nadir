use multibody::actuator::{Actuator, ActuatorSystem};
use nadir_result::{NadirResult, ResultManager};
use reaction_wheel::ReactionWheel;
use serde::{Deserialize, Serialize};

pub mod reaction_wheel;

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct SpacecraftActuators {
    pub rw: [Actuator<ReactionWheel>; 4],
}

impl ActuatorSystem for SpacecraftActuators {
    fn update(&mut self) {
        for rw in self.rw.iter_mut() {
            rw.update();
        }
    }

    fn initialize_results(&mut self, results: &mut ResultManager) {
        for rw in self.rw.iter_mut() {
            rw.new_result(results);
        }
    }

    fn write_results(&self, results: &mut ResultManager) {
        for rw in &self.rw {
            rw.write_result(results);
        }
    }

    fn state_derivative(&self, x0: &mut multibody::solver::SimStates) {
        for rw in &self.rw {
            rw.state_derivative(x0);
        }
    }

    fn state_vector_init(&mut self, x0: &mut multibody::solver::SimStates) {
        for rw in &mut self.rw {
            rw.state_vector_init(x0);
        }
    }

    fn state_vector_read(&mut self, x0: &multibody::solver::SimStates) {
        for rw in &mut self.rw {
            rw.state_vector_read(x0);
        }
    }
}
