use std::ops::{Add, Div, Mul};
pub trait Integrable:
    Add<Self,Output = Self>    
    + Mul<f64, Output = Self>
    + Div<f64,Output = Self>    
    + Sized
{
}

impl<T> Integrable for T where
    T: Add<T,Output = T> + Mul<f64,Output = T> + Div<f64,Output = T> + Sized
{
}
