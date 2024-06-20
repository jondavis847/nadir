pub mod solver;

use std::ops::{Add, Div, Mul};
pub trait Integrable:
    Add<Self, Output = Self> + Mul<f64, Output = Self> + Div<f64, Output = Self> + Sized + Clone
{
}
