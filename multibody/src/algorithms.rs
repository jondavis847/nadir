pub mod articulated_body_algorithm;
pub mod compositie_rigid_body;
pub mod recursive_newton_euler;

use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum MultibodyAlgorithm {
    //Auto,
    ArticulatedBody,
    CompositeRigidBody,
}