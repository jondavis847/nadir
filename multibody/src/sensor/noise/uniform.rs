use rand::rngs::SmallRng;
use rand_distr::{Distribution, Uniform};
use serde::{Deserialize, Serialize};
use uncertainty::{SimValue, Uncertainty};

use super::{NoiseErrors, NoiseTrait};

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct UniformBuilder {
    low: SimValue,
    high: SimValue,
}

impl UniformBuilder {
    pub fn new(low: f64, high: f64) -> Self {
        Self {
            low: SimValue::new(low),
            high: SimValue::new(high),
        }
    }
}

impl Uncertainty for UniformBuilder {
    type Output = UniformNoise;
    type Error = NoiseErrors;

    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let low = self.low.sample(nominal, rng);
        let high = self.high.sample(nominal, rng);
        Ok(UniformNoise::new(low, high))
    }
}

#[derive(Debug)]
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
