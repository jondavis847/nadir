use rand::rngs::{OsRng, StdRng};
use rand::{Rng, SeedableRng};
use rand_distr::{Distribution, Normal, NormalError, Uniform};
use serde::{Deserialize, Deserializer, Serialize};

#[derive(Debug, thiserror::Error)]
pub enum Error {
    #[error("{0}")]
    NormalError(#[from] NormalError),
}

pub trait Uncertainty {
    type Output;
    type Error;
    fn sample(&mut self) -> Result<Self::Output, Self::Error>;
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

    pub fn sample(&mut self) -> f64 {
        if let Some(dispersion) = &mut self.dispersion {
            dispersion.sample()
        } else {
            self.value
        }
    }

    /// The rand Distribution can be passed in as 'with_distribution(normal.into()) for example,
    /// where normal is a Normal<f64>, since we impl From<T: Distribution> for Distributions
    pub fn with_distribution(mut self, distribution: Distributions) -> Result<Self, Error> {
        let seed = OsRng.gen();
        let rng = StdRng::seed_from_u64(seed);
        self.dispersion = Some(Dispersion {
            nominal: self.value,
            distribution,
            rng,
            seed,
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

#[derive(Clone, Debug, Serialize)]
pub struct Dispersion {
    nominal: f64,
    distribution: Distributions,
    #[serde(skip)]
    rng: StdRng,
    seed: u64,
}

impl Dispersion {
    pub fn sample(&mut self) -> f64 {
        match self.distribution {
            Distributions::Normal(dist) => dist.sample(&mut self.rng),
            Distributions::Uniform(dist) => dist.sample(&mut self.rng),
        }
    }
}

impl<'de> Deserialize<'de> for Dispersion {
    fn deserialize<D>(deserializer: D) -> Result<Dispersion, D::Error>
    where
        D: Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct DispersionHelper {
            nominal: f64,
            distribution: Distributions,
            seed: u64,
        }

        let helper = DispersionHelper::deserialize(deserializer)?;

        Ok(Dispersion {
            nominal: helper.nominal,
            distribution: helper.distribution,
            seed: helper.seed,
            rng: StdRng::seed_from_u64(helper.seed),
        })
    }
}
