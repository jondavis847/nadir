use crate::{
    actuator::ActuatorModel, body::BodyConnection, solver::SimStateVector, HardwareBuffer,
};
use bytemuck::{Pod, Zeroable};
use nalgebra::{Vector3, Vector6};
use rand::rngs::SmallRng;
use rotations::{
    prelude::{QuaternionErrors, UnitQuaternion, UnitQuaternionBuilder},
    RotationTrait,
};
use serde::{Deserialize, Serialize};
use spatial_algebra::Force;
use thiserror::Error;
use uncertainty::{SimValue, Uncertainty};

use super::ActuatorErrors;

#[derive(Debug, Error)]
pub enum ReactionWheelErrors {
    #[error("knee speed must be less than or equal to max speed")]
    KneeGreaterThanMax,
    #[error("invalid command for reaction wheel")]
    InvalidCommand,
    #[error("coulomb friction should be greater than 0")]
    NegativeCoulomb,
    #[error("stiction should be greater than 0")]
    NegativeStiction,
    #[error("stiction threshold should be greater than 0")]
    NegativeStictionThreshold,
    #[error("viscous friction should be greater than 0")]
    NegativeViscous,
    #[error("windage friction should be greater than 0")]
    NegativeWindage,
    #[error("delay must be greater than 0")]
    NegativeDelay,
    #[error("inertia must be greater than 0")]
    NegativeMaxTorque,
    #[error("max torque must be greater than 0")]
    SmallInertia,
    #[error("{0}")]
    Quaternion(#[from] QuaternionErrors),
}

#[derive(Default, Debug, Clone, Copy, Pod, Zeroable)]
#[repr(C)]
pub struct ReactionWheelCommand {
    pub value: f64,
    pub command: u8,
    _padding: [u8; 7],
}

impl ReactionWheelCommand {
    const TORQUE: u8 = 0;
    const CURRENT: u8 = 1;
    const SPEED: u8 = 2;
    const ZERO: Self = Self {
        value: 0.0,
        command: 0,
        _padding: [0; 7],
    };
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct TorqueSpeedCurveBuilder {
    knee_speed: SimValue,
    max_speed: SimValue,
}

impl TorqueSpeedCurveBuilder {
    pub fn new(knee_speed: f64, max_speed: f64) -> Result<Self, ReactionWheelErrors> {
        if knee_speed > max_speed {
            return Err(ReactionWheelErrors::KneeGreaterThanMax);
        }
        Ok(Self {
            knee_speed: SimValue::new(knee_speed),
            max_speed: SimValue::new(max_speed),
        })
    }

    pub fn sample(&self, nominal: bool, rng: &mut SmallRng) -> TorqueSpeedCurve {
        TorqueSpeedCurve {
            knee_speed: self.knee_speed.sample(nominal, rng),
            max_speed: self.max_speed.sample(nominal, rng),
        }
    }
}

#[derive(Debug)]
struct TorqueSpeedCurve {
    knee_speed: f64, // rad/sec
    max_speed: f64,  // rad/sec
}

impl TorqueSpeedCurve {
    fn run(&self, torque: f64, speed: f64) -> f64 {
        if speed <= self.knee_speed {
            torque
        } else if speed > self.max_speed {
            0.0
        } else {
            torque * (1.0 - (speed - self.knee_speed) / (self.max_speed - self.knee_speed))
        }
    }
}
#[derive(Clone, Debug, Default, Deserialize, Serialize)]
pub struct ReactionWheelFrictionBuilder {
    stiction: SimValue,
    stiction_threshold: SimValue,
    coulomb: SimValue,
    viscous: SimValue,
    windage: SimValue,
}

impl ReactionWheelFrictionBuilder {
    pub fn sample(&self, nominal: bool, rng: &mut SmallRng) -> ReactionWheelFriction {
        ReactionWheelFriction {
            stiction: self.stiction.sample(nominal, rng),
            stiction_threshold: self.stiction_threshold.sample(nominal, rng),
            coulomb: self.coulomb.sample(nominal, rng),
            viscous: self.viscous.sample(nominal, rng),
            windage: self.windage.sample(nominal, rng),
        }
    }
}

#[derive(Debug)]
pub struct ReactionWheelFriction {
    stiction: f64,
    stiction_threshold: f64,
    coulomb: f64,
    viscous: f64,
    windage: f64,
}

impl ReactionWheelFriction {
    pub fn calculate_dynamic_friction(&self, velocity: f64) -> f64 {
        (-velocity.signum()) * self.coulomb
            - self.viscous * velocity
            - self.windage * velocity.powi(2)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
struct ReactionWheelParametersBuilder {
    delay: Option<SimValue>, //sec
    friction: ReactionWheelFrictionBuilder,
    inertia: SimValue, // kg-m^2
    misalignment: Option<UnitQuaternionBuilder>,
    torque_constant: SimValue,
    torque_max: Option<SimValue>,
    torque_speed_curve: Option<TorqueSpeedCurveBuilder>,
}

impl ReactionWheelParametersBuilder {
    pub fn new(inertia: f64, torque_constant: f64) -> Result<Self, ReactionWheelErrors> {
        if inertia < std::f64::EPSILON {
            return Err(ReactionWheelErrors::SmallInertia);
        }
        Ok(Self {
            delay: None,
            friction: ReactionWheelFrictionBuilder::default(),
            inertia: SimValue::new(inertia),
            misalignment: None,
            torque_constant: SimValue::new(torque_constant),
            torque_max: None,
            torque_speed_curve: None,
        })
    }

    pub fn sample(
        &self,
        nominal: bool,
        rng: &mut SmallRng,
    ) -> Result<ReactionWheelParameters, ReactionWheelErrors> {
        let delay = if let Some(delay) = &self.delay {
            Some(delay.sample(nominal, rng))
        } else {
            None
        };
        let misalignment = if let Some(misalignment) = &self.misalignment {
            Some(misalignment.sample(nominal, rng)?)
        } else {
            None
        };
        let torque_max = if let Some(torque_max) = &self.torque_max {
            Some(torque_max.sample(nominal, rng))
        } else {
            None
        };
        let torque_speed_curve = if let Some(torque_speed_curve) = &self.torque_speed_curve {
            Some(torque_speed_curve.sample(nominal, rng))
        } else {
            None
        };
        Ok(ReactionWheelParameters {
            delay,
            friction: self.friction.sample(nominal, rng),
            inertia: self.inertia.sample(nominal, rng),
            misalignment,
            torque_constant: self.torque_constant.sample(nominal, rng),
            torque_max,
            torque_speed_curve,
        })
    }
}

#[derive(Debug)]
struct ReactionWheelParameters {
    delay: Option<f64>, //sec
    friction: ReactionWheelFriction,
    inertia: f64, // kg-m^2
    misalignment: Option<UnitQuaternion>,
    torque_constant: f64,
    torque_max: Option<f64>,
    torque_speed_curve: Option<TorqueSpeedCurve>,
}

#[derive(Debug)]
pub struct ReactionWheelState {
    acceleration: f64, //rad/sec^2
    pub command: ReactionWheelCommand,
    current: f64,                // A
    momentum: f64,               //Nms
    momentum_body: Vector3<f64>, //Nms
    velocity: f64,               // m/s
    torque: f64,                 // Nm
    torque_body: Vector3<f64>,   // Nm
}

impl ReactionWheelState {
    pub fn new(initial_speed: f64, initial_momentum: f64) -> Self {
        Self {
            acceleration: 0.0,
            command: ReactionWheelCommand::default(),
            current: 0.0,
            momentum: initial_momentum,
            momentum_body: Vector3::zeros(),
            velocity: initial_speed,
            torque: 0.0,
            torque_body: Vector3::zeros(),
        }
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ReactionWheelBuilder {
    parameters: ReactionWheelParametersBuilder,
    initial_speed: SimValue,
}

impl ReactionWheelBuilder {
    pub fn new(inertia: f64, torque_constant: f64) -> Result<Self, ReactionWheelErrors> {
        Ok(Self {
            parameters: ReactionWheelParametersBuilder::new(inertia, torque_constant)?,
            initial_speed: SimValue::default(),
        })
    }

    pub fn set_coulomb(&mut self, coulomb: f64) -> Result<(), ReactionWheelErrors> {
        if coulomb < 0.0 {
            return Err(ReactionWheelErrors::NegativeCoulomb);
        }
        self.parameters.friction.coulomb.nominal = coulomb;
        Ok(())
    }

    pub fn with_coulomb(mut self, coulomb: f64) -> Result<Self, ReactionWheelErrors> {
        if coulomb < 0.0 {
            return Err(ReactionWheelErrors::NegativeCoulomb);
        }
        self.parameters.friction.coulomb.nominal = coulomb;
        Ok(self)
    }

    pub fn set_delay(&mut self, delay: f64) -> Result<(), ReactionWheelErrors> {
        if delay < 0.0 {
            return Err(ReactionWheelErrors::NegativeDelay);
        }
        if let Some(selfdelay) = &mut self.parameters.delay {
            selfdelay.nominal = delay;
        } else {
            self.parameters.delay = Some(SimValue::new(delay));
        }
        Ok(())
    }

    pub fn with_delay(mut self, delay: f64) -> Result<Self, ReactionWheelErrors> {
        if delay < 0.0 {
            return Err(ReactionWheelErrors::NegativeDelay);
        }
        if let Some(selfdelay) = &mut self.parameters.delay {
            selfdelay.nominal = delay;
        } else {
            self.parameters.delay = Some(SimValue::new(delay));
        }
        Ok(self)
    }

    pub fn set_speed(&mut self, speed: f64) {
        self.initial_speed.nominal = speed;
    }

    pub fn with_speed(mut self, speed: f64) -> Self {
        self.initial_speed.nominal = speed;
        self
    }

    pub fn set_misalignment(&mut self, misalignment: UnitQuaternionBuilder) {
        self.parameters.misalignment = Some(misalignment);
    }

    pub fn with_misalignment(mut self, misalignment: UnitQuaternionBuilder) -> Self {
        self.parameters.misalignment = Some(misalignment);
        self
    }

    pub fn set_stiction(&mut self, stiction: f64) -> Result<(), ReactionWheelErrors> {
        if stiction < 0.0 {
            return Err(ReactionWheelErrors::NegativeStiction);
        }
        self.parameters.friction.stiction.nominal = stiction;
        Ok(())
    }

    pub fn with_stiction(mut self, stiction: f64) -> Result<Self, ReactionWheelErrors> {
        if stiction < 0.0 {
            return Err(ReactionWheelErrors::NegativeStiction);
        }
        self.parameters.friction.stiction.nominal = stiction;
        Ok(self)
    }

    pub fn set_stiction_threshold(
        &mut self,
        stiction_threshold: f64,
    ) -> Result<(), ReactionWheelErrors> {
        if stiction_threshold < 0.0 {
            return Err(ReactionWheelErrors::NegativeStictionThreshold);
        }
        self.parameters.friction.stiction_threshold.nominal = stiction_threshold;
        Ok(())
    }

    pub fn with_stiction_threshold(
        mut self,
        stiction_threshold: f64,
    ) -> Result<Self, ReactionWheelErrors> {
        if stiction_threshold < 0.0 {
            return Err(ReactionWheelErrors::NegativeStictionThreshold);
        }
        self.parameters.friction.stiction_threshold.nominal = stiction_threshold;
        Ok(self)
    }

    pub fn set_torque_max(&mut self, torque_max: f64) -> Result<(), ReactionWheelErrors> {
        if torque_max < 0.0 {
            return Err(ReactionWheelErrors::NegativeMaxTorque);
        }
        if let Some(simval) = &mut self.parameters.torque_max {
            simval.nominal = torque_max;
        } else {
            self.parameters.torque_max = Some(SimValue::new(torque_max));
        }
        Ok(())
    }

    pub fn with_torque_max(mut self, torque_max: f64) -> Result<Self, ReactionWheelErrors> {
        if torque_max < 0.0 {
            return Err(ReactionWheelErrors::NegativeMaxTorque);
        }
        if let Some(simval) = &mut self.parameters.torque_max {
            simval.nominal = torque_max;
        } else {
            self.parameters.torque_max = Some(SimValue::new(torque_max));
        }
        Ok(self)
    }

    pub fn set_torque_speed_curve(
        &mut self,
        knee_speed: f64,
        max_speed: f64,
    ) -> Result<(), ReactionWheelErrors> {
        self.parameters.torque_speed_curve =
            Some(TorqueSpeedCurveBuilder::new(knee_speed, max_speed)?);
        Ok(())
    }

    pub fn with_torque_speed_curve(
        mut self,
        knee_speed: f64,
        max_speed: f64,
    ) -> Result<Self, ReactionWheelErrors> {
        self.parameters.torque_speed_curve =
            Some(TorqueSpeedCurveBuilder::new(knee_speed, max_speed)?);
        Ok(self)
    }

    pub fn set_viscous(&mut self, viscous: f64) -> Result<(), ReactionWheelErrors> {
        if viscous < 0.0 {
            return Err(ReactionWheelErrors::NegativeViscous);
        }
        self.parameters.friction.viscous.nominal = viscous;
        Ok(())
    }

    pub fn with_viscous(mut self, viscous: f64) -> Result<Self, ReactionWheelErrors> {
        if viscous < 0.0 {
            return Err(ReactionWheelErrors::NegativeViscous);
        }
        self.parameters.friction.viscous.nominal = viscous;
        Ok(self)
    }

    pub fn set_windage(&mut self, windage: f64) -> Result<(), ReactionWheelErrors> {
        if windage < 0.0 {
            return Err(ReactionWheelErrors::NegativeWindage);
        }
        self.parameters.friction.windage.nominal = windage;
        Ok(())
    }

    pub fn with_windage(mut self, windage: f64) -> Result<Self, ReactionWheelErrors> {
        if windage < 0.0 {
            return Err(ReactionWheelErrors::NegativeWindage);
        }
        self.parameters.friction.windage.nominal = windage;
        Ok(self)
    }
}

impl Uncertainty for ReactionWheelBuilder {
    type Error = ReactionWheelErrors;
    type Output = ReactionWheel;

    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let parameters = self.parameters.sample(nominal, rng)?;
        let initial_speed = self.initial_speed.sample(nominal, rng);
        let initial_momentum = initial_speed * parameters.inertia;
        let state = ReactionWheelState::new(initial_speed, initial_momentum);
        Ok(ReactionWheel { parameters, state })
    }
}

#[derive(Debug)]
pub struct ReactionWheel {
    parameters: ReactionWheelParameters,
    pub state: ReactionWheelState,
}

impl ActuatorModel for ReactionWheel {
    fn update(&mut self, connection: &BodyConnection) -> Result<(), ActuatorErrors> {
        // Determine initial torque based on command type
        let mut torque = match self.state.command.command {
            ReactionWheelCommand::TORQUE => {
                let desired_torque = self.state.command.value;
                // apply max torque limit if provided
                if let Some(torque_max) = self.parameters.torque_max {
                    desired_torque.clamp(-torque_max, torque_max)
                } else {
                    desired_torque
                }
            }
            ReactionWheelCommand::CURRENT => {
                self.parameters.torque_constant * self.state.command.value
            }
            ReactionWheelCommand::SPEED => {
                todo!()
            }
            _ => {
                return Err(ReactionWheelErrors::InvalidCommand.into());
            }
        };

        // Adjust torque based on torque-speed curve if provided
        if let Some(curve) = &self.parameters.torque_speed_curve {
            torque = curve.run(torque, self.state.velocity);
        }

        // Apply friction adjustments if provided
        if self.state.velocity.abs() > self.parameters.friction.stiction_threshold {
            // Apply dynamic friction for moving wheel
            torque += self
                .parameters
                .friction
                .calculate_dynamic_friction(self.state.velocity);
        } else {
            // Stiction torque opposes movement; ensure it doesn’t overpower torque
            torque = if torque.abs() > self.parameters.friction.stiction {
                let stiction_torque = -torque.signum() * self.parameters.friction.stiction;
                torque - stiction_torque
            } else {
                0.0 // Torque insufficient to overcome stiction, so no movement
            };
        }

        // Update state
        self.state.torque = torque;
        // equal and opposite
        self.state.torque_body = connection
            .transform
            .rotation
            .transform(&Vector3::new(0.0, 0.0, -torque));
        self.state.momentum_body =
            connection
                .transform
                .rotation
                .transform(&Vector3::new(0.0, 0.0, self.state.momentum));

        // apply misaligntment if applicable
        if let Some(misalignment) = &self.parameters.misalignment {
            self.state.torque_body = misalignment.transform(&self.state.torque_body);
            self.state.momentum_body = misalignment.transform(&self.state.momentum_body);
        }

        self.state.acceleration = torque / self.parameters.inertia;

        // Update body
        let mut body = connection.body.borrow_mut();
        body.state.internal_momentum_body += self.state.momentum_body;
        body.state.actuator_force_body += Force::from(Vector6::new(
            self.state.torque_body[0],
            self.state.torque_body[1],
            self.state.torque_body[2],
            0.0,
            0.0,
            0.0,
        ));
        Ok(())
    }

    fn result_content(&self, id: u32, results: &mut nadir_result::ResultManager) {
        results.write_record(
            id,
            &[
                self.state.acceleration.to_string(),
                self.state.current.to_string(),
                self.state.momentum.to_string(),
                self.state.momentum_body[0].to_string(),
                self.state.momentum_body[1].to_string(),
                self.state.momentum_body[2].to_string(),
                self.state.velocity.to_string(),
                self.state.torque.to_string(),
                self.state.torque_body[0].to_string(),
                self.state.torque_body[1].to_string(),
                self.state.torque_body[2].to_string(),
            ],
        );
    }

    fn result_headers(&self) -> &[&str] {
        &[
            "acceleration",
            "current",
            "momentum(wheel)",
            "momentum(body)[x]",
            "momentum(body)[y]",
            "momentum(body)[z]",
            "velocity",
            "torque(wheel)",
            "torque(body)[x]",
            "torque(body)[y]",
            "torque(body)[z]",
        ]
    }

    fn state_derivative(&self, derivative: &mut SimStateVector) {
        derivative.0[0] = self.state.acceleration;
    }

    fn state_vector_init(&self) -> SimStateVector {
        SimStateVector(vec![self.state.velocity])
    }

    fn state_vector_read(&mut self, state: &SimStateVector) {
        self.state.velocity = state.0[0];
        self.state.momentum = self.state.velocity * self.parameters.inertia;
    }

    fn read_command(&mut self, cmd: &HardwareBuffer) -> Result<(), ActuatorErrors> {
        self.state.command = cmd.read::<ReactionWheelCommand>()?;
        Ok(())
    }
}
