use rand::rngs::SmallRng;
use rand_distr::{Distribution, Uniform};
use serde::{Deserialize, Serialize};
use uncertainty::{UncertainValue, Uncertainty};

use super::{NoiseErrors, NoiseTrait};

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct UniformBuilder {
    low: UncertainValue,
    high: UncertainValue,
}

impl UniformBuilder {
    pub fn new(low: f64, high: f64) -> Self {
        Self {
            low: UncertainValue::new(low),
            high: UncertainValue::new(high),
        }
    }
}

impl Uncertainty for UniformBuilder {
    type Output = UniformNoise;
    type Error = NoiseErrors;

    fn sample(&self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let low = self.low.sample(nominal, rng);
        let high = self.high.sample(nominal, rng);
        Ok(UniformNoise::new(low, high)?)
    }
}

#[derive(Debug)]
pub struct UniformNoise {
    dist: Uniform<f64>,
}

impl UniformNoise {
    pub fn new(min: f64, max: f64) -> Result<Self, NoiseErrors> {
        let dist = Uniform::new(min, max)?;
        Ok(Self { dist })
    }
}

impl NoiseTrait for UniformNoise {
    fn sample(&self, rng: &mut SmallRng) -> f64 {
        self.dist.sample(rng)
    }
}
