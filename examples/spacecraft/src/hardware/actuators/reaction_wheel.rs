use multibody::{actuator::ActuatorModel, body::BodyConnection, solver::SimStateVector};
use nalgebra::{Vector3, Vector6};
use rotations::{Rotation, RotationTrait};
use serde::{Deserialize, Serialize};
use spatial_algebra::Force;
use thiserror::Error;

#[derive(Debug, Error)]
pub enum ReactionWheelErrors {}

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
}

#[derive(Clone, Copy, Debug, Deserialize, Serialize)]
pub struct ReactionWheelState {
    acceleration: f64, //rad/sec^2
    pub command: ReactionWheelCommands,
    current: f64,                // A
    momentum: f64,               //Nms
    momentum_body: Vector3<f64>, //Nms
    velocity: f64,               // m/s
    torque: f64,                 // Nm
    torque_body: Vector3<f64>,   // Nm
}

impl ReactionWheelState {
    pub fn new(initial_momentum: f64) -> Self {
        Self {
            acceleration: 0.0,
            command: ReactionWheelCommands::Torque(0.0),
            current: 0.0,
            momentum: initial_momentum,
            momentum_body: Vector3::zeros(),
            velocity: 0.0,
            torque: 0.0,
            torque_body: Vector3::zeros(),
        }
    }
}

#[derive(Copy, Clone, Debug, Deserialize, Serialize)]
pub struct ReactionWheel {
    parameters: ReactionWheelParameters,
    pub state: ReactionWheelState,
}

impl ReactionWheel {
    pub fn new(
        inertia: f64,
        torque_max: f64,
        initial_momentum: f64,
    ) -> Result<Self, ReactionWheelErrors> {
        let parameters = ReactionWheelParameters {
            inertia,
            torque_max,
            friction: None,
            torque_constant: None,
            delay: None,
            misalignment: None,
            torque_speed_curve: None,
        };
        Ok(Self {
            parameters,
            state: ReactionWheelState::new(initial_momentum),
        })
    }
    #[allow(dead_code)]
    pub fn with_delay(mut self, delay: f64) -> Self {
        self.parameters.delay = Some(delay);
        self
    }
    #[allow(dead_code)]
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
    #[allow(dead_code)]
    pub fn with_misalignment(mut self, misalignment: Rotation) -> Self {
        self.parameters.misalignment = Some(misalignment);
        self
    }
    #[allow(dead_code)]
    pub fn with_torque_constant(mut self, torque_constant: f64) -> Self {
        self.parameters.torque_constant = Some(torque_constant);
        self
    }
    #[allow(dead_code)]
    pub fn with_torque_speed_curve(mut self, knee_speed: f64, max_speed: f64) -> Self {
        let torque_speed_curve = TorqueSpeedCurve {
            knee_speed,
            max_speed,
        };
        self.parameters.torque_speed_curve = Some(torque_speed_curve);
        self
    }
}

impl ActuatorModel for ReactionWheel {
    type Command = ReactionWheelCommands;
    fn update(&mut self, connection: &mut BodyConnection) {
        // Determine initial torque based on command type
        let mut torque = match self.state.command {
            ReactionWheelCommands::Current(current) => {
                if let Some(kt) = &self.parameters.torque_constant {
                    kt * current
                } else {
                    eprintln!(
                        "Need torque constant (kt) to process current command. Doing nothing"
                    );
                    0.0
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
        let transform = &connection.transform;
        self.state.torque = torque;
        // equal and opposite
        self.state.torque_body = transform
            .rotation
            .transform(Vector3::new(0.0, 0.0, -torque));
        self.state.momentum_body =
            transform
                .rotation
                .transform(Vector3::new(0.0, 0.0, self.state.momentum));
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
        SimStateVector(vec![self.state.acceleration])
    }

    fn state_vector_read(&mut self, state: &SimStateVector) {
        self.state.velocity = state.0[0];
        self.state.momentum = self.state.velocity * self.parameters.inertia;
    }
}
