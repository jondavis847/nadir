use std::fmt::Debug;

use nalgebra::Vector3;
use rand::rngs::SmallRng;
use rand_distr::{Distribution, Normal as RNormal, NormalError, uniform::Error as UniformError};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum UncertaintyErrors {
    #[error("{0}")]
    NormalError(#[from] NormalError),
    #[error("{0}")]
    UniformError(#[from] UniformError),
    #[error("'low' can't be greater than 'high' for uniform distribution")]
    LowGreaterThanHigh,
}

/// We wrap the Uniform<T> from rand_distr since it doesnt error on a low > high
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Uniform(rand_distr::Uniform<f64>);
impl Uniform {
    pub fn new(low: f64, high: f64) -> Result<Self, UncertaintyErrors> {
        if low > high {
            return Err(UncertaintyErrors::LowGreaterThanHigh);
        }
        Ok(Self(
            rand_distr::Uniform::new(low, high)?,
        ))
    }
}

/// Wrapper for normal, pretty much done just for error propagation
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Normal(RNormal<f64>);
impl Normal {
    pub fn new(mean: f64, std: f64) -> Result<Self, UncertaintyErrors> {
        Ok(Self(RNormal::new(mean, std)?))
    }
}

pub trait Uncertainty {
    type Output;
    type Error: Debug;
    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error>;
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct UncertainValue {
    pub nominal: f64,
    pub dispersion: Option<Dispersion>,
}

impl UncertainValue {
    pub fn new(nominal: f64) -> Self {
        Self { nominal, dispersion: None }
    }

    pub fn sample(&self, nominal: bool, rng: &mut SmallRng) -> f64 {
        if nominal {
            return self.nominal;
        }
        if let Some(dispersion) = &self.dispersion {
            dispersion.sample(rng)
        } else {
            self.nominal
        }
    }

    /// The rand Distribution can be passed in as 'with_distribution(normal.into()) for example,
    /// where normal is a Normal<f64>, since we impl From<T: Distribution> for Distributions
    pub fn with_distribution(
        mut self,
        distribution: Distributions,
    ) -> Result<Self, UncertaintyErrors> {
        self.dispersion = Some(Dispersion { distribution });
        Ok(self)
    }

    /// The rand Distribution can be passed in as 'with_distribution(normal.into()) for example,
    /// where normal is a Normal<f64>, since we impl From<T: Distribution> for Distributions
    pub fn set_distribution(
        &mut self,
        distribution: Distributions,
    ) -> Result<(), UncertaintyErrors> {
        self.dispersion = Some(Dispersion { distribution });
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Distributions {
    Normal(Normal),
    Uniform(Uniform),
}

impl From<Normal> for Distributions {
    fn from(normal: Normal) -> Self {
        Distributions::Normal(normal)
    }
}

impl From<Uniform> for Distributions {
    fn from(uniform: Uniform) -> Self {
        Distributions::Uniform(uniform)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct Dispersion {
    pub distribution: Distributions,
}

impl Dispersion {
    pub fn sample(&self, rng: &mut SmallRng) -> f64 {
        match &self.distribution {
            Distributions::Normal(dist) => dist
                .0
                .sample(rng),
            Distributions::Uniform(dist) => dist
                .0
                .sample(rng),
        }
    }
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct SimVector3 {
    pub x: UncertainValue,
    pub y: UncertainValue,
    pub z: UncertainValue,
}

impl Uncertainty for SimVector3 {
    type Error = ();
    type Output = Vector3<f64>;

    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let x = self
            .x
            .sample(nominal, rng);
        let y = self
            .y
            .sample(nominal, rng);
        let z = self
            .z
            .sample(nominal, rng);
        Ok(Vector3::new(x, y, z))
    }
}
