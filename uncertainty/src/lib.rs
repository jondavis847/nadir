use rand::rngs::ThreadRng;
use rand_distr::{Normal, NormalError, Uniform};
use serde::{Deserialize, Serialize};
use thiserror::Error;

#[derive(Debug, Error)]
pub enum UncertaintyErrors {
    #[error("{0}")]
    NormalError(#[from] NormalError),
}

pub trait Uncertainty {
    fn sample(&mut self) -> Self;
}

#[derive(Clone, Debug, Default, Serialize, Deserialize)]
pub struct UncertainValue {
    dist: Distributions,
}

impl UncertainValue {
    pub fn new_float(value: f64) -> Self {
        Self {
            dist: Distributions::None(value),
        }
    }

    pub fn new_normal(mean: f64, std: f64) -> Result<Self, UncertaintyErrors> {
        Ok(Self {
            dist: Distributions::Normal {
                dist: Normal::new(mean, std)?,
                rng: rand::thread_rng(),
            },
        })
    }

    pub fn new_uniform(lower: f64, upper: f64) -> Self {
        Self {
            dist: Distributions::Uniform {
                dist: Uniform::new(lower, upper),
                rng: rand::thread_rng(),
            },
        }
    }
}

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum Distributions {
    None(f64),
    Normal { dist: Normal<f64>, rng: ThreadRng },
    Uniform { dist: Uniform<f64>, rng: ThreadRng },
}
