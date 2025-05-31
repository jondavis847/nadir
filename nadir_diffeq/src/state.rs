use std::{
    fmt::Debug,
    ops::{Add, Mul, Sub},
};

use tolerance::Tolerance;

pub trait OdeState: Clone + Debug {
    type Value: StateValue;
    type Tolerance: Tolerance;
}

pub trait StateValue:
    Copy + Clone + Debug + Sized + Default + Mul<f64> + Add<Self::Derivative> + Sub<Self>
{
    type Derivative: Clone + Copy + Debug + Mul<f64> + Sized + Default;
}
