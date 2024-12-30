use rand::rngs::SmallRng;
use rand_distr::{Distribution, Uniform};
use serde::{Deserialize, Serialize};

use super::NoiseTrait;

#[derive(Debug, Clone, Copy, Deserialize, Serialize)]
pub struct UniformNoise {
    dist: Uniform<f64>,
}

impl UniformNoise {
    pub fn new(min: f64, max: f64) -> Self {
        let dist = Uniform::from(min..max);
        Self { dist }
    }
}

impl NoiseTrait for UniformNoise {
    fn sample(&self, rng: &mut SmallRng) -> f64 {
        self.dist.sample(rng)
    }
}
