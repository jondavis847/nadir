// THIS HAS BEEN DEPRECATED AND WE NOW KEEP THE SOLVER
// IN MULTIBODY SINCE IT GREATLY SIMPLIFIES THE USE CASE
// THIS IS JUST HERE FOR REFERENCE

pub mod solver;
use std::fmt::Debug;

use std::ops::{AddAssign, MulAssign};

pub trait Integrable:
    for<'a> AddAssign<&'a Self> + MulAssign<f64> + Sized + Clone + Debug    
{
}

impl<T> Integrable for T where
    T: for<'a> AddAssign<&'a T>
        + MulAssign<f64>        
        + Sized
        + Clone
        + Debug     
{
}

// function signature is (dx,x,p,t), where dx is some mutable container for the result
pub trait OdeFunction<P, T>: FnMut(&mut T, &T, &Option<P>, f64) {}

impl<P, T, F> OdeFunction<P, T> for F where F: FnMut(&mut T, &T, &Option<P>, f64) {}

pub trait CallbackFunction<P, T>: FnMut(&T, &Option<P>, f64) -> (T,Option<P>,f64) {}

impl<P, T, F> CallbackFunction<P, T> for F where F: FnMut(&T, &Option<P>, f64) -> (T,Option<P>,f64) {}

