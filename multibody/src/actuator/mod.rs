use nadir_diffeq::{
    saving::{StateWriter, StateWriterBuilder, WriterId, WriterManager},
    state::state_vector::StateVector,
};
use rand::rngs::SmallRng;
use reaction_wheel::{ReactionWheel, ReactionWheelBuilder, ReactionWheelErrors};
use serde::{Deserialize, Serialize};
use std::{fmt::Debug, path::PathBuf};
use thiserror::Error;
use thruster::{Thruster, ThrusterBuilder, ThrusterErrors};
use transforms::Transform;
use uncertainty::Uncertainty;

use crate::{
    BufferError, HardwareBuffer,
    body::{BodyConnection, BodyConnectionBuilder},
    system::Id,
};

pub mod reaction_wheel;
pub mod thruster;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct ActuatorBuilder {
    pub name: String,
    pub model: ActuatorModelBuilders,
    pub connection: Option<BodyConnectionBuilder>,
}

impl ActuatorBuilder {
    pub fn new(name: &str, model: ActuatorModelBuilders) -> Self {
        Self {
            name: name.to_string(),
            model,
            connection: None,
        }
    }
    pub fn connect_body(&mut self, body: Id, transform: Transform) {
        self.connection = Some(BodyConnectionBuilder::new(body, transform));
    }

    pub fn sample(
        &self,
        nominal: bool,
        rng: &mut SmallRng,
        connection: BodyConnection,
    ) -> Result<Actuator, ActuatorErrors> {
        let model = self.model.sample(nominal, rng)?;
        Ok(Actuator {
            name: self.name.clone(),
            model,
            connection,
            writer_id: None,
            state_start: 0,
            state_end: 0,
        })
    }
}

#[derive(Debug, Error)]
pub enum ActuatorErrors {
    #[error("actuator '{0}' is already connected to body '{1}'")]
    AlreadyConnectedToAnotherBody(String, String),
    #[error("actuator '{0}' is already connected to that body")]
    AlreadyConnectedToThisBody(String),
    #[error("{0}")]
    BufferError(#[from] BufferError),
    #[error("{0}")]
    ReactionWheelErrors(#[from] ReactionWheelErrors),
    #[error("{0}")]
    ThrusterErrors(#[from] ThrusterErrors),
}

pub trait ActuatorModel {
    fn update(&mut self, connection: &BodyConnection) -> Result<(), ActuatorErrors>;
    /// Populates derivative with the appropriate values for the actuator state derivative
    fn state_derivative(&self, _derivative: &mut [f64]) {}
    /// Initializes a vector of f64 values representing state vector for the ODE integration
    fn state_vector_init(&self) -> StateVector {
        StateVector::new(vec![])
    }
    /// Reads a state vector into the sim state
    fn state_vector_read(&mut self, state: &[f64]);
    fn read_command(&mut self, buffer: &HardwareBuffer) -> Result<(), ActuatorErrors>;
    fn writer_headers(&self) -> &[&str];
    fn writer_save_fn(&self, writer: &mut StateWriter);
}

#[derive(Debug)]
pub struct Actuator {
    pub name: String,
    pub model: ActuatorModels,
    pub connection: BodyConnection,
    /// Id of the result writer in sys.writers
    writer_id: Option<WriterId>,
    state_start: usize,
    state_end: usize,
}

impl Actuator {
    pub fn update(&mut self) -> Result<(), ActuatorErrors> {
        self.model.update(&self.connection)
    }

    pub fn state_vector_init(&mut self, x0: &mut StateVector) {
        self.state_start = x0.len();
        x0.extend(&self.model.state_vector_init());
        self.state_end = x0.len();
    }

    pub fn state_vector_read(&mut self, x0: &StateVector) {
        let state = &x0[self.state_start..self.state_end];
        self.model.state_vector_read(state);
    }

    pub fn state_derivative(&self, x0: &mut StateVector) {
        let derivative = &mut x0[self.state_start..self.state_end];
        self.model.state_derivative(derivative);
    }

    pub fn read_command(&mut self, buffer: &HardwareBuffer) -> Result<(), ActuatorErrors> {
        self.model.read_command(buffer)
    }

    pub fn writer_init_fn(&mut self, manager: &mut WriterManager) {
        let rel_path = PathBuf::new().join("actuators").join(&self.name);
        let headers = self.model.writer_headers();
        let writer = StateWriterBuilder::new(headers.len(), rel_path);
        self.writer_id = Some(manager.add_writer(writer));
    }

    pub fn writer_save_fn(&self, manager: &mut WriterManager) {
        if let Some(id) = &self.writer_id {
            if let Some(writer) = manager.writers.get_mut(id) {
                self.model.writer_save_fn(writer);
            }
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum ActuatorModelBuilders {
    ReactionWheel(ReactionWheelBuilder),
    Thruster(ThrusterBuilder),
}

impl ActuatorModelBuilders {
    pub fn sample(
        &self,
        nominal: bool,
        rng: &mut SmallRng,
    ) -> Result<ActuatorModels, ActuatorErrors> {
        match self {
            ActuatorModelBuilders::ReactionWheel(builder) => {
                Ok(ActuatorModels::ReactionWheel(builder.sample(nominal, rng)?))
            }
            ActuatorModelBuilders::Thruster(builder) => {
                Ok(ActuatorModels::Thruster(builder.sample(nominal, rng)?))
            }
        }
    }
}

impl From<ReactionWheelBuilder> for ActuatorModelBuilders {
    fn from(builder: ReactionWheelBuilder) -> Self {
        ActuatorModelBuilders::ReactionWheel(builder)
    }
}

impl From<ThrusterBuilder> for ActuatorModelBuilders {
    fn from(builder: ThrusterBuilder) -> Self {
        ActuatorModelBuilders::Thruster(builder)
    }
}

#[derive(Debug)]
pub enum ActuatorModels {
    ReactionWheel(ReactionWheel),
    Thruster(Thruster),
}

impl ActuatorModel for ActuatorModels {
    fn writer_save_fn(&self, writer: &mut StateWriter) {
        match self {
            ActuatorModels::ReactionWheel(act) => act.writer_save_fn(writer),
            ActuatorModels::Thruster(act) => act.writer_save_fn(writer),
        }
    }

    fn writer_headers(&self) -> &[&str] {
        match self {
            ActuatorModels::ReactionWheel(act) => act.writer_headers(),
            ActuatorModels::Thruster(act) => act.writer_headers(),
        }
    }

    fn state_derivative(&self, derivative: &mut [f64]) {
        match self {
            ActuatorModels::ReactionWheel(act) => act.state_derivative(derivative),
            ActuatorModels::Thruster(act) => act.state_derivative(derivative),
        }
    }

    fn state_vector_init(&self) -> StateVector {
        match self {
            ActuatorModels::ReactionWheel(act) => act.state_vector_init(),
            ActuatorModels::Thruster(act) => act.state_vector_init(),
        }
    }

    fn state_vector_read(&mut self, state: &[f64]) {
        match self {
            ActuatorModels::ReactionWheel(act) => act.state_vector_read(state),
            ActuatorModels::Thruster(act) => act.state_vector_read(state),
        }
    }

    fn update(&mut self, connection: &BodyConnection) -> Result<(), ActuatorErrors> {
        match self {
            ActuatorModels::ReactionWheel(act) => act.update(connection),
            ActuatorModels::Thruster(act) => act.update(connection),
        }
    }

    fn read_command(&mut self, cmd: &HardwareBuffer) -> Result<(), ActuatorErrors> {
        match self {
            ActuatorModels::ReactionWheel(act) => act.read_command(cmd),
            ActuatorModels::Thruster(act) => act.read_command(cmd),
        }
    }
}
