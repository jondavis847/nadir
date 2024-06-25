use crate::{body::BodySim, joint::JointSim};
use spatial_algebra::{Acceleration, Force, SpatialInertia, Velocity};

#[derive(Clone, Copy, Debug, Default)]
pub struct AbaCache {
    pub vj: Velocity,
    pub v: Velocity,
    pub c: Acceleration,
    pub p_big_a: Force, //gah rust naming conventions warn on pA
    pub p_lil_a: Force,
    pub a: Acceleration,
    pub a_prime: Acceleration,
    pub inertia_articulated: SpatialInertia,
}
pub trait ArticulatedBodyAlgorithm {
    fn first_pass(&mut self, v_ij: Velocity, f_ob: &Force);
    fn second_pass(&mut self, inner_is_base: bool) -> Option<(SpatialInertia, Force)>;    
    fn third_pass(&mut self, a_ij: Acceleration);

    fn get_v(&self) -> &Velocity;

    fn get_p_big_a(&self) -> &Force; //gah rust naming conventions warn on pA

    fn get_a(&self) -> &Acceleration;

    fn add_inertia_articulated(&mut self, inertia: SpatialInertia);
    fn add_p_big_a(&mut self, force: Force);

}
