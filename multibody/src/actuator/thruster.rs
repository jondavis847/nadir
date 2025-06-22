use crate::{BufferError, HardwareBuffer, actuator::ActuatorModel, body::BodyConnection};
use bytemuck::{Pod, Zeroable};
use nadir_diffeq::saving::StateWriter;
use nalgebra::{Vector3, Vector6};
use rand::rngs::SmallRng;
use rotations::{
    RotationTrait,
    prelude::{QuaternionErrors, UnitQuaternion, UnitQuaternionBuilder},
};
use serde::{Deserialize, Serialize};
use spatial_algebra::Force;
use thiserror::Error;
use uncertainty::{SimValue, Uncertainty};

use super::ActuatorErrors;

#[derive(Debug, Error)]
pub enum ThrusterErrors {
    #[error("{0}")]
    BufferError(#[from] BufferError),
    #[error("invalid command for thruster")]
    InvalidCommand,
    #[error("{0}")]
    Quaternion(#[from] QuaternionErrors),
    #[error("thruster force must be greater than 0.0")]
    NegativeDelay,
}

#[derive(Clone, Copy, Debug, Pod, PartialEq, Zeroable)]
#[repr(C)]
pub struct ThrusterCommand(u8);

impl ThrusterCommand {
    const OFF: Self = Self(0);
    const ON: Self = Self(1);
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct ThrusterParametersBuilder {
    delay: Option<SimValue>, //sec
    misalignment: Option<UnitQuaternionBuilder>,
    force: SimValue,
}

impl ThrusterParametersBuilder {
    pub fn new(force: f64) -> Result<Self, ThrusterErrors> {
        Ok(Self {
            delay: None,
            misalignment: None,
            force: SimValue::new(force),
        })
    }

    pub fn sample(
        &self,
        nominal: bool,
        rng: &mut SmallRng,
    ) -> Result<ThrusterParameters, ThrusterErrors> {
        let misalignment = if let Some(misalignment) = &self.misalignment {
            Some(misalignment.sample(nominal, rng)?)
        } else {
            None
        };
        Ok(ThrusterParameters {
            misalignment,
            force: self.force.sample(nominal, rng),
        })
    }
}

#[derive(Debug)]
struct ThrusterParameters {
    misalignment: Option<UnitQuaternion>,
    force: f64,
}

#[derive(Debug)]
pub struct ThrusterState {
    pub command: ThrusterCommand,
    force: f64,                // N
    force_body: Vector3<f64>,  // N
    torque_body: Vector3<f64>, //Nm
}

impl ThrusterState {
    pub fn new() -> Self {
        Self {
            command: ThrusterCommand::OFF,
            force: 0.0,
            force_body: Vector3::zeros(),
            torque_body: Vector3::zeros(),
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ThrusterBuilder {
    parameters: ThrusterParametersBuilder,
}

impl ThrusterBuilder {
    pub fn new(force: f64) -> Result<Self, ThrusterErrors> {
        Ok(Self { parameters: ThrusterParametersBuilder::new(force)? })
    }

    pub fn set_delay(&mut self, delay: f64) -> Result<(), ThrusterErrors> {
        if delay < 0.0 {
            return Err(ThrusterErrors::NegativeDelay);
        }
        if let Some(selfdelay) = &mut self.parameters.delay {
            selfdelay.nominal = delay;
        } else {
            self.parameters.delay = Some(SimValue::new(delay));
        }
        Ok(())
    }

    pub fn with_delay(mut self, delay: f64) -> Result<Self, ThrusterErrors> {
        if delay < 0.0 {
            return Err(ThrusterErrors::NegativeDelay);
        }
        if let Some(selfdelay) = &mut self.parameters.delay {
            selfdelay.nominal = delay;
        } else {
            self.parameters.delay = Some(SimValue::new(delay));
        }
        Ok(self)
    }

    pub fn set_misalignment(&mut self, misalignment: UnitQuaternionBuilder) {
        self.parameters.misalignment = Some(misalignment);
    }

    pub fn with_misalignment(mut self, misalignment: UnitQuaternionBuilder) -> Self {
        self.parameters.misalignment = Some(misalignment);
        self
    }
}

impl Uncertainty for ThrusterBuilder {
    type Error = ThrusterErrors;
    type Output = Thruster;

    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let parameters = self.parameters.sample(nominal, rng)?;
        let state = ThrusterState::new();
        Ok(Thruster { parameters, state })
    }
}

#[derive(Debug)]
pub struct Thruster {
    parameters: ThrusterParameters,
    pub state: ThrusterState,
}

impl ActuatorModel for Thruster {
    fn update(&mut self, connection: &BodyConnection) -> Result<(), ActuatorErrors> {
        // Determine initial torque based on command type
        let force = match self.state.command {
            ThrusterCommand::ON => self.parameters.force,
            ThrusterCommand::OFF => 0.0,
            _ => return Err(ThrusterErrors::InvalidCommand.into()),
        };

        // Update state
        self.state.force = force;
        // equal and opposite
        self.state.force_body = connection
            .transform
            .rotation
            .transform(&Vector3::new(force, 0.0, 0.0));

        // apply misalignment if applicable
        if let Some(misalignment) = &self.parameters.misalignment {
            self.state.force_body = misalignment.transform(&self.state.force_body);
        }

        // Update body
        let mut body = connection.body.borrow_mut();
        let moment_arm = connection.transform.translation.vec();
        let torque_body = moment_arm.cross(&self.state.force_body);
        body.state.actuator_force_body += Force::from(Vector6::new(
            torque_body[0],
            torque_body[1],
            torque_body[2],
            self.state.force_body[0],
            self.state.force_body[1],
            self.state.force_body[2],
        ));
        Ok(())
    }

    fn writer_save_fn(&self, writer: &mut StateWriter) {
        writer.float_buffer[0] = self.state.command.0 as f64;
        writer.float_buffer[1] = self.state.force;
        writer.float_buffer[2] = self.state.force_body[0];
        writer.float_buffer[3] = self.state.force_body[1];
        writer.float_buffer[4] = self.state.force_body[2];
        writer.float_buffer[5] = self.state.torque_body[0];
        writer.float_buffer[6] = self.state.torque_body[1];
        writer.float_buffer[7] = self.state.torque_body[2];
        writer.write_record().unwrap();
    }

    fn writer_headers(&self) -> &[&str] {
        &[
            "command",
            "force(thruster)",
            "force(body)[x]",
            "force(body)[y]",
            "force(body)[z]",
            "torque(body)[x]",
            "torque(body)[y]",
            "torque(body)[z]",
        ]
    }

    fn state_vector_read(&mut self, _state: &[f64]) {}

    fn read_command(&mut self, cmd: &HardwareBuffer) -> Result<(), ActuatorErrors> {
        self.state.command = cmd.read::<ThrusterCommand>()?;
        Ok(())
    }
}
