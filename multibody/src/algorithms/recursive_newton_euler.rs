use spatial_algebra::{Acceleration, Force, Velocity};


#[derive(Clone, Copy, Debug, Default)]
pub struct RneCache {    
    pub f: Force,    
}

pub trait RecursiveNewtonEuler {
    fn rne_first_pass(&mut self, a_ij: Acceleration, v_ij: Velocity, f_ob: &Force, use_qddot: bool);
    fn rne_second_pass(&mut self);
}