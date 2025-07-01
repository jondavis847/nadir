use crate::{HardwareBuffer, actuator::ActuatorModel, body::BodyConnection};
use nadir_diffeq::{saving::StateWriter, state::state_vector::StateVector};
use nalgebra::{Vector3, Vector6};
use rand::rngs::SmallRng;
use rotations::{
    RotationTrait,
    prelude::{QuaternionErrors, UnitQuaternion, UnitQuaternionBuilder},
};
use serde::{Deserialize, Serialize};
use spatial_algebra::Force;
use thiserror::Error;
use uncertainty::{UncertainValue, Uncertainty};

use super::ActuatorErrors;

#[derive(Debug, Error)]
pub enum MagneticTorquerErrors {
    #[error("invalid command for reaction wheel")]
    InvalidCommand,
    #[error("{0}")]
    QuaternionErrors(#[from] QuaternionErrors),
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct MagneticTorquerParametersBuilder {
    current_to_moment: UncertainValue,
    misalignment: Option<UnitQuaternionBuilder>,
}

impl MagneticTorquerParametersBuilder {
    pub fn new(current_to_moment: UncertainValue) -> Result<Self, MagneticTorquerErrors> {
        Ok(Self { current_to_moment, misalignment: None })
    }

    pub fn sample(
        &self,
        nominal: bool,
        rng: &mut SmallRng,
    ) -> Result<MagneticTorquerParameters, MagneticTorquerErrors> {
        let current_to_moment = self
            .current_to_moment
            .sample(nominal, rng);
        let misalignment = if let Some(misalignment) = &self.misalignment {
            Some(misalignment.sample(nominal, rng)?)
        } else {
            None
        };

        Ok(MagneticTorquerParameters { current_to_moment, misalignment })
    }
}

#[derive(Debug)]
struct MagneticTorquerParameters {
    current_to_moment: f64,
    misalignment: Option<UnitQuaternion>,
}

#[derive(Debug)]
pub struct MagneticTorquerState {
    pub command: f64,
    current: f64, // A
    moment: f64,
    torque_act: Vector3<f64>,
    torque_body: Vector3<f64>,
}

impl MagneticTorquerState {
    pub fn new() -> Self {
        Self {
            command: 0.0,
            current: 0.0,
            moment: 0.0,
            torque_act: Vector3::zeros(),
            torque_body: Vector3::zeros(),
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MagneticTorquerBuilder {
    parameters: MagneticTorquerParametersBuilder,
}

impl MagneticTorquerBuilder {
    pub fn new(current_to_moment: UncertainValue) -> Result<Self, MagneticTorquerErrors> {
        Ok(Self {
            parameters: MagneticTorquerParametersBuilder::new(current_to_moment)?,
        })
    }

    pub fn set_misalignment(&mut self, misalignment: UnitQuaternionBuilder) {
        self.parameters
            .misalignment = Some(misalignment);
    }

    pub fn with_misalignment(mut self, misalignment: UnitQuaternionBuilder) -> Self {
        self.parameters
            .misalignment = Some(misalignment);
        self
    }
}

impl Uncertainty for MagneticTorquerBuilder {
    type Error = MagneticTorquerErrors;
    type Output = MagneticTorquer;

    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let parameters = self
            .parameters
            .sample(nominal, rng)?;

        Ok(MagneticTorquer { parameters, state: MagneticTorquerState::new() })
    }
}

#[derive(Debug)]
pub struct MagneticTorquer {
    parameters: MagneticTorquerParameters,
    pub state: MagneticTorquerState,
}

impl ActuatorModel for MagneticTorquer {
    fn update(&mut self, connection: &BodyConnection) -> Result<(), ActuatorErrors> {
        // Determine moment
        let moment = self
            .state
            .current
            * self
                .parameters
                .current_to_moment;

        let magnetic_field_body = connection
            .body
            .borrow()
            .state
            .magnetic_field_body;
        let mut magnetic_field_torquer = connection
            .transform
            .rotation
            .transform(&magnetic_field_body);

        // Apply misalignment
        if let Some(misalignment) = &self
            .parameters
            .misalignment
        {
            magnetic_field_torquer = misalignment.transform(&magnetic_field_torquer);
        }

        let torque_act = Vector3::new(0.0, 0.0, moment).cross(&magnetic_field_torquer);
        let torque_body = if let Some(misalignment) = &self
            .parameters
            .misalignment
        {
            // inverse now to get back to body frame
            connection
                .transform
                .rotation
                .inv()
                .transform(
                    &misalignment
                        .inv()
                        .transform(&torque_act),
                )
        } else {
            // no misalignment
            connection
                .transform
                .rotation
                .inv()
                .transform(&torque_act)
        };

        // Update state
        self.state
            .moment = moment;
        self.state
            .torque_act = torque_act;
        self.state
            .torque_body = torque_body;

        // Update body
        let mut body = connection
            .body
            .borrow_mut();
        body.state
            .actuator_force_body += Force::from(Vector6::new(
            self.state
                .torque_body[0],
            self.state
                .torque_body[1],
            self.state
                .torque_body[2],
            0.0,
            0.0,
            0.0,
        ));
        Ok(())
    }

    fn writer_save_fn(&self, writer: &mut StateWriter) {
        writer.float_buffer[0] = self
            .state
            .command;
        writer.float_buffer[1] = self
            .state
            .current;
        writer.float_buffer[2] = self
            .state
            .torque_act[0];
        writer.float_buffer[3] = self
            .state
            .torque_act[1];
        writer.float_buffer[4] = self
            .state
            .torque_act[2];
        writer.float_buffer[5] = self
            .state
            .torque_body[0];
        writer.float_buffer[6] = self
            .state
            .torque_body[1];
        writer.float_buffer[7] = self
            .state
            .torque_body[2];
        writer
            .write_record()
            .unwrap();
    }

    fn writer_headers(&self) -> &[&str] {
        &[
            "command",
            "current",
            "torque(act)[x]",
            "torque(act)[y]",
            "torque(act)[z]",
            "torque(body)[x]",
            "torque(body)[x]",
            "torque(body)[x]",
        ]
    }

    fn state_derivative(&self, _derivative: &mut [f64]) {}

    fn state_vector_init(&self) -> StateVector {
        StateVector::new(vec![])
    }

    fn state_vector_read(&mut self, _state: &[f64]) {}

    fn read_command(&mut self, cmd: &HardwareBuffer) -> Result<(), ActuatorErrors> {
        self.state
            .command = cmd.read::<f64>()?;
        Ok(())
    }
}
