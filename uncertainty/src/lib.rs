use rand::rngs::StdRng;
use rand_distr::{Distribution, Normal, NormalError, Uniform};
use serde::{Deserialize, Serialize};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{0}")]
    NormalError(#[from] NormalError),
}

pub trait Uncertainty {
    type Output;
    type Error;
    fn sample(&mut self, rng: &mut StdRng) -> Result<Self::Output, Self::Error>;
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

    pub fn sample(&mut self, rng: &mut StdRng) -> f64 {
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
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Distributions {
    Normal(Normal<f64>),
    Uniform(Uniform<f64>),
}

impl From<Normal<f64>> for Distributions {
    fn from(normal: Normal<f64>) -> Self {
        Distributions::Normal(normal)
    }
}

impl From<Uniform<f64>> for Distributions {
    fn from(uniform: Uniform<f64>) -> Self {
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
        match self.distribution {
            Distributions::Normal(dist) => dist.sample(rng),
            Distributions::Uniform(dist) => dist.sample(rng),
        }
    }
}

pub struct SimVector3 {
    pub x: SimValue,
    pub y: SimValue,
    pub z: SimValue,
}
