use nalgebra::{DMatrix, DVector};
use serde::{Serialize, Deserialize};
use spatial_algebra::SpatialInertia;

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct CrbCache { 
    pub c: DVector<f64>,
    pub h: DMatrix<f64>,       
}

impl CrbCache {
    pub fn new(n: usize) -> Self {
        Self {
            c: DVector::<f64>::zeros(n),
            h: DMatrix::<f64>::zeros(n,n),
        }
    }
}

pub trait CompositeRigidBody { 
    fn add_ic(&mut self, ic: SpatialInertia);
    fn reset_ic(&mut self);    
    fn get_crb_index(&self) -> usize;
    fn get_ic(&self) -> SpatialInertia;
    fn set_crb_index(&mut self, n: usize);
    fn set_c(&self, c: &mut DVector<f64>);
    fn set_h(&self, h: &mut DMatrix<f64>);
}