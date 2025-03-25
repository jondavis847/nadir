use nalgebra::Vector3;
use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal, NormalError};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{0}")]
    NormalError(#[from] NormalError),
    #[error("{0}")]
    UniformError(#[from] UniformError),
}

#[derive(Debug, Error)]
pub enum UniformError {
    #[error("'low' can't be greater than 'high' for uniform distribution")]
    LowGreaterThanHigh,
}

/// We wrap the Uniform<T> from rand_distr since it doesnt error on a low > high
#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct Uniform(rand_distr::Uniform<f64>);
impl Uniform {
    pub fn new(low: f64, high: f64) -> Result<Self, UniformError> {
        if low > high {
            return Err(UniformError::LowGreaterThanHigh);
        }
        Ok(Self(rand_distr::Uniform::new(low, high)))
    }
}

pub trait Uncertainty {
    type Output;
    type Error;
    fn sample(&mut self, nominal: bool, rng: &mut StdRng) -> Result<Self::Output, Self::Error>;
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct SimValue {
    pub value: f64,
    pub dispersion: Option<Dispersion>,
}

impl SimValue {
    pub fn new(value: f64) -> Self {
        Self {
            value,
            dispersion: None,
        }
    }

    pub fn sample(&mut self, nominal: bool, rng: &mut StdRng) -> f64 {
        if nominal {
            return self.value;
        }
        if let Some(dispersion) = &mut self.dispersion {
            dispersion.sample(rng)
        } else {
            self.value
        }
    }

    /// The rand Distribution can be passed in as 'with_distribution(normal.into()) for example,
    /// where normal is a Normal<f64>, since we impl From<T: Distribution> for Distributions
    pub fn with_distribution(mut self, distribution: Distributions) -> Result<Self, Error> {
        self.dispersion = Some(Dispersion {
            nominal: self.value,
            distribution,
        });
        Ok(self)
    }

    /// The rand Distribution can be passed in as 'with_distribution(normal.into()) for example,
    /// where normal is a Normal<f64>, since we impl From<T: Distribution> for Distributions
    pub fn set_distribution(&mut self, distribution: Distributions) -> Result<(), Error> {
        self.dispersion = Some(Dispersion {
            nominal: self.value,
            distribution,
        });
        Ok(())
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Distributions {
    Normal(Normal<f64>),
    Uniform(Uniform),
}

impl From<Normal<f64>> for Distributions {
    fn from(normal: Normal<f64>) -> Self {
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
    nominal: f64,
    distribution: Distributions,
}

impl Dispersion {
    pub fn sample(&mut self, rng: &mut StdRng) -> f64 {
        match &self.distribution {
            Distributions::Normal(dist) => dist.sample(rng),
            Distributions::Uniform(dist) => dist.0.sample(rng),
        }
    }
}

#[derive(Debug, Default, Clone, Serialize, Deserialize)]
pub struct SimVector3 {
    pub x: SimValue,
    pub y: SimValue,
    pub z: SimValue,
}

impl Uncertainty for SimVector3 {
    type Error = ();
    type Output = Vector3<f64>;

    fn sample(&mut self, nominal: bool, rng: &mut StdRng) -> Result<Self::Output, Self::Error> {
        let x = self.x.sample(nominal, rng);
        let y = self.y.sample(nominal, rng);
        let z = self.z.sample(nominal, rng);
        Ok(Vector3::new(x, y, z))
    }
}
