use indexmap::IndexMap;
use nalgebra::Vector3;
use rotations::RotationTrait;

use crate::{body::Body, system::Id};

/// Defines a mechanism for mass, momentum, and energy calculations
/// The frame of the mechanism is the same as the frame of the first body
#[derive(Debug, Clone, Default)]
pub struct Mechanism {
    bodies: Vec<Id>,
    state: MechanismState,
}

impl Mechanism {
    pub fn new() -> Self {
        Self::default()
    }
    // TODO: should this be a BodyId type so we cant add the wrong type?
    pub fn with_body(mut self, body: Id) -> Self {
        self.bodies.push(body.clone());
        self
    }

    pub fn update(&mut self, bodies: &IndexMap<Id, Body>) {
        self.state.kinetic_energy = 0.0;
        self.state.potential_energy = 0.0;
        self.state.total_energy = 0.0;
        self.state.linear_momentum = Vector3::zeros();
        self.state.angular_momentum = Vector3::zeros();

        for (i, id) in self.bodies.iter().enumerate() {
            if let Some(body) = bodies.get(id) {
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
                        let first_body = bodies.get(&self.bodies[0]).unwrap();
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
