pub trait ActuatorProcessing {
    type ActuatorCommand;
    type ActuatorState;
    fn process_actuator(&mut self, command: Self::ActuatorCommand);
    fn get_actuator_state(&self) -> Self::ActuatorState;
}