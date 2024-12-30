use rand::rngs::SmallRng;
use rand_distr::{Distribution, Normal};
use serde::{Deserialize, Serialize};

use super::NoiseTrait;

#[derive(Debug, Clone, Copy, Serialize, Deserialize)]
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
        self.dist.sample(rng)
    }
}
