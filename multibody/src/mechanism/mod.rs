use crate::body::BodyRef;
use nalgebra::Vector3;
use rotations::RotationTrait;

/// Defines a mechanism for mass, momentum, and energy calculations
/// The frame of the mechanism is the same as the frame of the first body
#[derive(Debug, Clone, Default)]
pub struct Mechanism {
    bodies: Vec<BodyRef>,
    state: MechanismState,
}

impl Mechanism {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn with_body(mut self, body: &BodyRef) -> Self {
        self.bodies.push(body.clone());
        self
    }

    pub fn update(&mut self) {
        self.state.kinetic_energy = 0.0;
        self.state.potential_energy = 0.0;
        self.state.total_energy = 0.0;
        self.state.linear_momentum = Vector3::zeros();
        self.state.angular_momentum = Vector3::zeros();

        for (i, bodyref) in self.bodies.iter().enumerate() {
            let body = bodyref.borrow();
            match i {
                0 => {
                    // mechanism frame is the same as first body
                    self.state.linear_momentum += body.state.linear_momentum_body;
                    self.state.angular_momentum += body.state.angular_momentum_body;
                }
                _ => {
                    // rotate momentum to mechanism frame
                    // can't just use momentum base because it includes orbital angular momentum in addition to body angular momentum
                    // use body momentum, rotated to base by this body transform, then rotated to mechanism by first body transform
                    let first_body = self.bodies[0].borrow();
                    let base_to_mechanism = first_body.state.attitude_base;
                    let body_to_base = body.state.attitude_base.inv();
                    let body_to_mechanism = base_to_mechanism * body_to_base;

                    self.state.linear_momentum +=
                        body_to_mechanism.transform(&body.state.linear_momentum_body);
                    self.state.angular_momentum +=
                        body_to_mechanism.transform(&body.state.angular_momentum_body);
                }
            }
            self.state.kinetic_energy += body.state.kinetic_energy;
            self.state.potential_energy += body.state.potential_energy;
        }

        self.state.total_energy = self.state.kinetic_energy + self.state.potential_energy;
    }
}

#[derive(Debug, Clone, Default)]
pub struct MechanismState {
    kinetic_energy: f64,
    potential_energy: f64,
    total_energy: f64,
    linear_momentum: Vector3<f64>,
    angular_momentum: Vector3<f64>,
}
