pub mod articulated_body_algorithm;
use serde::{Serialize, Deserialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum MultibodyAlgorithm {
    //Auto,
    ArticulatedBody,
    //CompositeRigidBody,
}