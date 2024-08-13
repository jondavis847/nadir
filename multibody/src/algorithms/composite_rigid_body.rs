use nalgebra::{DMatrix, DVector};

#[derive(Clone, Debug, Default)]
pub struct CrbCache { 
    c: DVector<f64>,
    h: DMatrix<f64>,       
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
    fn set_crb_index(&mut self, n: usize);
}