use spatial_algebra::{Force,Motion};

#[derive(Clone,Copy,Debug,Default)]
pub struct AbaCache {
    vj: Motion,
    c: Motion,
    p_big_a: Force, //gah rust naming conventions warn on pA
    p_lil_a: Force,    
    a: Motion,
    aprime: Motion,    
}

pub trait ArticulatedBodyAlgorithm {
    fn first_pass(&mut self);
    fn second_pass(&mut self);
    fn third_pass(&mut self);
}