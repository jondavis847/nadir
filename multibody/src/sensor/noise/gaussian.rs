use rand::rngs::SmallRng;
use rand_distr::{Distribution, Normal};
use serde::{Deserialize, Serialize};
use uncertainty::{UncertainValue, Uncertainty};

use super::{NoiseErrors, NoiseTrait};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct GaussianBuilder {
    mean: UncertainValue,
    sigma: UncertainValue,
}

impl GaussianBuilder {
    pub fn new(mean: f64, sigma: f64) -> Self {
        Self {
            mean: UncertainValue::new(mean),
            sigma: UncertainValue::new(sigma),
        }
    }
}
impl Uncertainty for GaussianBuilder {
    type Output = GaussianNoise;
    type Error = NoiseErrors;

    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let mean = self
            .mean
            .sample(nominal, rng);
        let sigma = self
            .sigma
            .sample(nominal, rng);
        Ok(GaussianNoise::new(
            mean, sigma,
        ))
    }
}

#[derive(Debug)]
pub struct GaussianNoise {
    dist: Normal<f64>,
}

impl GaussianNoise {
    pub fn new(mean: f64, sigma: f64) -> Self {
        let dist = Normal::new(mean, sigma).unwrap();
        Self { dist }
    }
}

impl NoiseTrait for GaussianNoise {
    fn sample(&self, rng: &mut SmallRng) -> f64 {
        self.dist
            .sample(rng)
    }
}
