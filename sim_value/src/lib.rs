use num_traits::{One, Zero};
use std::ops::{Add, Div, Mul, Neg, Sub};
use std::convert::From;
//NOTE: Do we want to use Pow instead of making our own Sqrt?

pub trait SimValue:
    Abs
    + Add<Output = Self>    
    + Copy
    + Default
    + Div<Output = Self>    
    + From<f64>
    + From<f32>
    + Mul<Output = Self>    
    + Neg<Output = Self>
    + One
    + PartialOrd
    + Sub<Output = Self>    
    + Sqrt
    + Trigonometry
    + Zero
{
    const EPSILON: Self;
}

impl SimValue for f64 {
    const EPSILON: f64 = f64::EPSILON;
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

pub trait Trigonometry {
    fn cos(self) -> Self;
    fn sin(self) -> Self;
}

impl Trigonometry for f64 {
    fn cos(self) -> Self {
        f64::cos(self)
    }
    fn sin(self) -> Self {
        f64::sin(self)
    }
}


impl Trigonometry for f32 {
    fn cos(self) -> Self {
        f32::cos(self)
    }
    fn sin(self) -> Self {
        f32::sin(self)
    }
}
