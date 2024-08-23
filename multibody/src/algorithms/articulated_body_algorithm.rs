use serde::{Deserialize, Serialize};
use spatial_algebra::{Acceleration, Force, SpatialInertia, Velocity};
#[derive(Clone, Copy, Debug, Default, Serialize, Deserialize)]
pub struct AbaCache {
    pub c: Acceleration,
    pub p_big_a: Force, //gah rust naming conventions warn on pA
    pub p_lil_a: Force,
    pub a_prime: Acceleration,
    pub inertia_articulated: SpatialInertia,
}
pub trait ArticulatedBodyAlgorithm {
    fn aba_first_pass(&mut self, v_ij: Velocity);
    fn aba_second_pass(&mut self, inner_is_base: bool) -> Option<(SpatialInertia, Force)>;
    fn aba_third_pass(&mut self, a_ij: Acceleration);
    fn get_p_big_a(&self) -> Force; //gah rust naming conventions warn on pA
    fn add_inertia_articulated(&mut self, inertia: SpatialInertia);
    fn add_p_big_a(&mut self, force: Force);
}
