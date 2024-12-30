pub mod articulated_body_algorithm;
pub mod composite_rigid_body;
pub mod recursive_newton_euler;

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
pub enum MultibodyAlgorithm {
    //Auto,
    ArticulatedBody,
    CompositeRigidBody,
}
