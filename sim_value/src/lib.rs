use num_traits::{One, Zero};
use std::ops::{Add, Div, Mul, Neg, Sub};
//NOTE: Do we want to use Pow instead of making our own Sqrt?

pub trait SimValue:
    Abs
    + Add<Output = Self>
    + Copy
    + Div<Output = Self>    
    + Mul<Output = Self>
    + Neg<Output = Self>    
    + One
    + PartialOrd
    + Sub<Output = Self>
    + Sqrt
    + Zero
{
    const EPSILON: Self;
}
impl<T> SimValue for T where
    T: Abs
        + Add<Output = Self>
        + Copy
        + Div<Output = Self>        
        + Mul<Output = Self>
        + Neg<Output = Self>
        + One
        + PartialOrd
        + Sub<Output = Self>
        + Sqrt
        + Zero
{
    const EPSILON:T = T::EPSILON;
}

/// Trait defining a square root operation.
pub trait Sqrt {
    /// Returns the square root of the value.
    fn sqrt(self) -> Self;
}

/// Implementation of the `Sqrt` trait for `f64`.
impl Sqrt for f64 {
    fn sqrt(self) -> Self {
        f64::sqrt(self)
    }
}

/// Implementation of the `Sqrt` trait for `f64`.
impl Sqrt for f32 {
    fn sqrt(self) -> Self {
        f32::sqrt(self)
    }
}

pub trait Abs {
    fn abs(self) -> Self;
}

impl Abs for f64 {
    fn abs(self) -> Self {
        f64::abs(self)
    }
}

impl Abs for f32 {
    fn abs(self) -> Self {
        f32::abs(self)
    }
}

