pub mod solver;
use std::fmt::Debug;

use std::ops::{AddAssign, DivAssign, MulAssign};

pub trait Integrable:
    for<'a> AddAssign<&'a mut Self> + for<'a> MulAssign<f64> + for<'a> DivAssign<f64> + Sized + Clone + Debug
{
}

impl<T> Integrable for T where
    T: for<'a> AddAssign<&'a mut T> + for<'a> MulAssign<f64> + for<'a> DivAssign<f64> + Sized + Clone  + Debug
{
}

// function signature is (dx,x,p,t), where dx is some mutable container for the result
pub trait OdeFunction<P,T>: Fn(&mut T, &T, &Option<P>, f64) {}

impl<P, T, F> OdeFunction<P, T> for F
where
    F: Fn(&mut T, &T, &Option<P>, f64),
{}
