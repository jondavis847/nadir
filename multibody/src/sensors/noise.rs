pub mod gaussian;
pub mod uniform;

use gaussian::GaussianNoise;
use rand::{rngs::SmallRng, Rng, SeedableRng};
use uniform::UniformNoise;
use serde::{Serialize, Deserialize};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum NoiseModels {
    None,
    Gaussian(GaussianNoise),
    Uniform(UniformNoise),
}

#[derive(Clone, Debug, Serialize)]
pub struct Noise {
    model: NoiseModels,
    #[serde(skip)]
    rng: SmallRng,
    seed: u64,    
}

impl Noise {
    pub fn new(model: NoiseModels) -> Self {
        let seed = rand::thread_rng().gen();
        let rng = SmallRng::seed_from_u64(seed);
        Self { rng, seed, model }
    }

    pub fn new_seed(&mut self) {
        let seed = rand::thread_rng().gen();
        self.seed = seed;
    }
}

// SmallRng does not impl Deserialize, just recreate when loading the model
impl<'de> Deserialize<'de> for Noise {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct NoiseHelper {
            model: NoiseModels,
            seed: u64,
        }

        let helper = NoiseHelper::deserialize(deserializer)?;
        Ok(Noise {
            model: helper.model,
            rng: SmallRng::seed_from_u64(helper.seed),
            seed: helper.seed,
        })
    }
}

pub trait NoiseTrait {
    fn sample(&self, rng: &mut SmallRng) -> f64;
}
