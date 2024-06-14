use spatial_algebra::{Force, Motion};

#[derive(Clone, Copy, Debug, Default)]
pub struct AbaCache {
    pub vj: Motion,
    pub v: Motion,
    pub c: Motion,
    pub p_big_a: Force, //gah rust naming conventions warn on pA
    pub p_lil_a: Force,
    pub a: Motion,
    pub a_prime: Motion,
}
pub trait ArticulatedBodyAlgorithm {
    fn first_pass(&mut self);
    fn second_pass(&mut self);
    fn third_pass(&mut self);

    fn get_v(&self) -> Motion;

    fn get_p_big_a(&self) -> Force; //gah rust naming conventions warn on pA

    fn get_a(&self) -> Motion;
}
