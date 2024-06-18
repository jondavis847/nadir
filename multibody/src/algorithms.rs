pub mod articulated_body_algorithm;

#[derive(Debug, Clone)]
pub enum MultibodyAlgorithm {
    Auto,
    ArticulatedBody,
    CompositeRigidBody,
}