pub mod solver;
use std::fmt::Debug;

use std::ops::{Add, AddAssign, Div, DivAssign, Mul, MulAssign};

pub trait Integrable:
    Add<Self, Output = Self> + Mul<f64, Output = Self> + Div<f64, Output = Self> + Sized + Clone + Debug
{
}

impl<T> Integrable for T where
    T: Add<T, Output = Self>
        + Mul<f64, Output = Self>
        + Div<f64, Output = Self>
        + Sized
        + Clone
        + Debug
{
}

pub trait IntegrableIP:
    for<'a> AddAssign<&'a Self>
    + for<'a> MulAssign<f64>
    + for<'a> DivAssign<f64>
    + Sized
    + Clone
    + Debug
{
}

impl<T> IntegrableIP for T where
    T: for<'a> AddAssign<&'a T>
        + for<'a> MulAssign<f64>
        + for<'a> DivAssign<f64>
        + Sized
        + Clone
        + Debug
{
}

// function signature is (dx,x,p,t), where dx is some mutable container for the result
pub trait OdeFunctionIP<P, T>: Fn(&mut T, &T, &Option<P>, f64) {}

impl<P, T, F> OdeFunctionIP<P, T> for F where F: Fn(&mut T, &T, &Option<P>, f64) {}

pub trait OdeFunction<P, T>: FnMut(&T, &Option<P>, f64) -> T {}

impl<P, T, F> OdeFunction<P, T> for F where F: FnMut(&T, &Option<P>, f64) -> T {}

pub trait CallbackFunction<P, T>: FnMut(&T, &Option<P>, f64) -> (T,Option<P>,f64) {}

impl<P, T, F> CallbackFunction<P, T> for F where F: FnMut(&T, &Option<P>, f64) -> (T,Option<P>,f64) {}

