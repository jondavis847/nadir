use nalgebra::{DMatrix, DVector};

#[derive(Clone, Debug, Default)]
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
    fn get_crb_index(&self) -> usize;
    fn set_crb_index(&mut self, n: usize);
}