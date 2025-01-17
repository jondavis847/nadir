pub mod gaussian;
pub mod uniform;

use gaussian::GaussianNoise;
use nalgebra::Vector3;
use rand::{rngs::SmallRng, Rng, SeedableRng};
use rotations::{axis_angle::AxisAngle, quaternion::UnitQuaternion};
use serde::{Deserialize, Serialize};
use uniform::UniformNoise;

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum NoiseModels {
    None,
    Gaussian(GaussianNoise),
    Uniform(UniformNoise),
}

impl NoiseTrait for NoiseModels {
    fn sample(&self, rng: &mut SmallRng) -> f64 {
        match self {
            NoiseModels::None => 0.0,
            NoiseModels::Gaussian(noise) => noise.sample(rng),
            NoiseModels::Uniform(noise) => noise.sample(rng),
        }
    }
}

#[derive(Clone, Debug, Serialize)]
pub struct Noise {
    model: NoiseModels,
    seed: u64,
    #[serde(skip)]
    rng: SmallRng,
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
        self.rng = SmallRng::seed_from_u64(seed);
    }

    pub fn sample(&mut self) -> f64 {
        self.model.sample(&mut self.rng)
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

#[derive(Clone, Debug, Serialize)]
pub struct QuaternionNoise {
    magnitude_noise: Noise,
    axis_seed: u64,
    #[serde(skip)]
    axis_rng: SmallRng,
}

impl QuaternionNoise {
    pub fn new(noise: NoiseModels) -> Self {
        let magnitude_noise = Noise::new(noise);
        let axis_seed = rand::thread_rng().gen();
        let axis_rng = SmallRng::seed_from_u64(axis_seed);

        Self {
            magnitude_noise,
            axis_seed,
            axis_rng,
        }
    }

    pub fn sample(&mut self) -> UnitQuaternion {
        // Generate a small random angle for the noise
        let noise_angle = self.magnitude_noise.sample();

        let random_axis = Vector3::new(
            self.axis_rng.gen(),
            self.axis_rng.gen(),
            self.axis_rng.gen(),
        )
        .normalize();

        // Create a noise quaternion from the random axis and noise angle
        // unwrap should be safe since we call .normalize()
        let noise_axis_angle = AxisAngle::new(noise_angle, random_axis).unwrap();
        UnitQuaternion::from(&noise_axis_angle)
    }
}

// SmallRng does not impl Deserialize, just recreate when loading the model
impl<'de> Deserialize<'de> for QuaternionNoise {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct NoiseHelper {
            magnitude_noise: Noise,
            axis_seed: u64,
        }

        let helper = NoiseHelper::deserialize(deserializer)?;
        Ok(QuaternionNoise {
            magnitude_noise: helper.magnitude_noise,
            axis_seed: helper.axis_seed,
            axis_rng: SmallRng::seed_from_u64(helper.axis_seed),
        })
    }
}
