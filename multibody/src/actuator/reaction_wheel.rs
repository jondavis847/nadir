use nalgebra::Vector3;
use rotations::{Rotation, RotationTrait};
use serde::{Deserialize, Serialize};
use thiserror::Error;
use transforms::Transform;

#[derive(Debug, Error)]
pub enum ReactionWheelErrors {
    #[error("torque constant is not specified but is required for current commanding")]
    NeedKtForCurrentCommanding,
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub enum ReactionWheelCommands {
    Current(f64),
    Speed(f64),
    Torque(f64),
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
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

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
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

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
struct ReactionWheelParameters {
    delay: Option<f64>, //sec
    friction: Option<ReactionWheelFriction>,
    inertia: f64, // kg-m^2
    misalignment: Option<Rotation>,
    torque_constant: Option<f64>,
    torque_max: f64,
    torque_speed_curve: Option<TorqueSpeedCurve>,
    transform: Transform,
}

#[derive(Clone, Copy, Debug, Default, Deserialize, Serialize)]
pub struct ReactionWheelState {
    acceleration: f64,           //rad/sec^2
    current: f64,                // A
    momentum: f64,               //Nms
    momentum_body: Vector3<f64>, //Nms
    velocity: f64,               // m/s
    torque: f64,                 // Nm
    torque_body: Vector3<f64>,   // Nm
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct ReactionWheel {
    name: String,
    parameters: ReactionWheelParameters,
    state: ReactionWheelState,
}

impl ReactionWheel {
    pub fn new(
        name: String,
        inertia: f64,
        torque_max: f64,
        transform: Transform,
    ) -> Result<Self, ReactionWheelErrors> {
        let parameters = ReactionWheelParameters {
            inertia,
            torque_max,
            transform,
            friction: None,
            torque_constant: None,
            delay: None,
            misalignment: None,
            torque_speed_curve: None,
        };
        Ok(Self {
            name,
            parameters,
            state: ReactionWheelState::default(),
        })
    }

    pub fn with_delay(mut self, delay: f64) -> Self {
        self.parameters.delay = Some(delay);
        self
    }

    pub fn with_friction(
        mut self,
        stiction: f64,
        stiction_threshold: f64,
        coulomb: f64,
        viscous: f64,
        windage: f64,
    ) -> Self {
        let friction = ReactionWheelFriction {
            stiction,
            stiction_threshold,
            coulomb,
            viscous,
            windage,
        };
        self.parameters.friction = Some(friction);
        self
    }

    pub fn with_misalignment(mut self, misalignment: Rotation) -> Self {
        self.parameters.misalignment = Some(misalignment);
        self
    }

    pub fn with_torque_constant(mut self, torque_constant: f64) -> Self {
        self.parameters.torque_constant = Some(torque_constant);
        self
    }

    pub fn with_torque_speed_curve(mut self, knee_speed: f64, max_speed: f64) -> Self {
        let torque_speed_curve = TorqueSpeedCurve {
            knee_speed,
            max_speed,
        };
        self.parameters.torque_speed_curve = Some(torque_speed_curve);
        self
    }

    pub fn process_command(
        &mut self,
        command: ReactionWheelCommands,
    ) -> Result<(), ReactionWheelErrors> {
        // Determine initial torque based on command type
        let mut torque = match command {
            ReactionWheelCommands::Current(current) => {
                if let Some(kt) = &self.parameters.torque_constant {
                    kt * current
                } else {
                    return Err(ReactionWheelErrors::NeedKtForCurrentCommanding);
                }
            }
            ReactionWheelCommands::Speed(target_speed) => {
                // Simple proportional control: full torque in the direction needed to reach target
                if self.state.velocity < target_speed {
                    self.parameters.torque_max
                } else if self.state.velocity > target_speed {
                    -self.parameters.torque_max
                } else {
                    0.0
                }
            }
            ReactionWheelCommands::Torque(requested_torque) => {
                requested_torque.clamp(-self.parameters.torque_max, self.parameters.torque_max)
            }
        };

        // Adjust torque based on torque-speed curve if provided
        if let Some(curve) = &self.parameters.torque_speed_curve {
            torque = curve.run(torque, self.state.velocity);
        }

        // Apply friction adjustments if provided
        if let Some(friction) = &self.parameters.friction {
            if self.state.velocity.abs() > friction.stiction_threshold {
                // Apply dynamic friction for moving wheel
                torque += friction.calculate_dynamic_friction(self.state.velocity);
            } else {
                // Stiction torque opposes movement; ensure it doesn’t overpower torque
                torque = if torque.abs() > friction.stiction {
                    let stiction_torque = -torque.signum() * friction.stiction;
                    torque - stiction_torque
                } else {
                    0.0 // Torque insufficient to overcome stiction, so no movement
                };
            }
        }

        // Update state
        self.state.torque = torque;
        self.state.torque_body = self
            .parameters
            .transform
            .rotation
            .transform(Vector3::new(0.0, 0.0, torque));
        self.state.momentum_body = self.parameters.transform.rotation.transform(Vector3::new(
            0.0,
            0.0,
            self.state.momentum,
        ));
        self.state.acceleration = torque / self.parameters.inertia;

        Ok(())
    }
}

pub struct ReactionWheelResult(pub Vec<ReactionWheelState>);
