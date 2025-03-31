pub mod gaussian;
pub mod uniform;

use gaussian::{GaussianBuilder, GaussianNoise};
use nalgebra::Vector3;
use rand::{rngs::SmallRng, Rng, SeedableRng};
use rotations::{axis_angle::AxisAngle, quaternion::UnitQuaternion};
use serde::{Deserialize, Serialize};
use uncertainty::{SimValue, Uncertainty};
use uniform::{UniformBuilder, UniformNoise};

#[derive(Clone, Debug, Serialize, Deserialize)]
pub enum NoiseModelBuilders {
    None,
    Gaussian(GaussianBuilder),
    Uniform(UniformBuilder),
}

impl Uncertainty for NoiseModelBuilders {
    type Output = NoiseModels;
    type Error = ();

    fn sample(&mut self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let model = match self {
            NoiseModelBuilders::None => NoiseModels::None,
            NoiseModelBuilders::Gaussian(model) => {
                let noise = model.sample(nominal, rng)?;
                NoiseModels::Gaussian(noise)
            }
            NoiseModelBuilders::Uniform(model) => {
                let noise = model.sample(nominal, rng)?;
                NoiseModels::Uniform(noise)
            }
        };
        Ok(model)
    }
}

#[derive(Debug)]
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

#[derive(Clone, Debug, Serialize, Deserialize)]
pub struct NoiseBuilder {
    model: NoiseModelBuilders,
}

impl NoiseBuilder {
    pub fn new_normal(mean: f64, sigma: f64) -> Self {
        Self {
            model: NoiseModelBuilders::Gaussian(GaussianBuilder::new(mean, sigma)),
        }
    }

    pub fn new_uniform(low: f64, high: f64) -> Self {
        Self {
            model: NoiseModelBuilders::Uniform(UniformBuilder::new(low, high)),
        }
    }
}

impl Uncertainty for NoiseBuilder {
    type Output = Noise;
    type Error = ();

    fn sample(&mut self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let model = match &mut self.model {
            NoiseModelBuilders::None => NoiseModels::None,
            NoiseModelBuilders::Gaussian(model) => {
                let noise = model.sample(nominal, rng)?;
                NoiseModels::Gaussian(noise)
            }
            NoiseModelBuilders::Uniform(model) => {
                let noise = model.sample(nominal, rng)?;
                NoiseModels::Uniform(noise)
            }
        };
        let seed = rng.gen();
        let rng = SmallRng::seed_from_u64(seed);
        Ok(Noise { model, rng, seed })
    }
}

#[derive(Debug)]
pub struct Noise {
    model: NoiseModels,
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
        self.rng = SmallRng::seed_from_u64(seed);
    }

    pub fn sample(&mut self) -> f64 {
        self.model.sample(&mut self.rng)
    }
}

pub trait NoiseTrait {
    fn sample(&self, rng: &mut SmallRng) -> f64;
}

pub struct QuaternioNoiseBuilder {
    magnitude_noise: NoiseBuilder,
}

impl Uncertainty for QuaternioNoiseBuilder {
    type Output = QuaternionNoise;
    type Error = ();

    fn sample(&mut self, nominal: bool, rng: &mut SmallRng) -> Result<Self::Output, Self::Error> {
        let magnitude_noise = self.magnitude_noise.sample(nominal, rng)?;
        let axis_seed = rng.gen();
        let axis_rng = SmallRng::seed_from_u64(axis_seed);
        Ok(QuaternionNoise {
            magnitude_noise,
            axis_seed,
            axis_rng,
        })
    }
}

#[derive(Debug)]
pub struct QuaternionNoise {
    magnitude_noise: Noise,
    axis_seed: u64,
    axis_rng: SmallRng,
}

impl QuaternionNoise {
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
