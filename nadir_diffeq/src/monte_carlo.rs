use rand::{Rng, SeedableRng, rngs::SmallRng};
use std::error::Error;
use uncertainty::Uncertainty;

use crate::{OdeModel, OdeProblem, state::OdeState};

pub trait MonteCarloModel: Uncertainty<Output = Self::Model>
where
    Self::Model: OdeModel,
{
    type Model;
}

pub trait MonteCarloState: Uncertainty<Output = Self::State>
where
    Self::State: OdeState,
{
    type State;
}

pub struct MonteCarloProblem<ModelBuilder, StateBuilder>
where
    ModelBuilder: MonteCarloModel,
    StateBuilder: MonteCarloState,
{
    model_builder: ModelBuilder,
    state_builder: StateBuilder,
    nruns: usize,
    seed: u64,
}

impl<ModelBuilder, StateBuilder> MonteCarloProblem<ModelBuilder, StateBuilder>
where
    ModelBuilder: MonteCarloModel,
    StateBuilder: MonteCarloState,
{
    /// Creates a new `MonteCarloProblem` instance with the specified configuration.
    pub fn new(model_builder: ModelBuilder, state_builder: StateBuilder, nruns: usize) -> Self {
        let mut thread_rng = rand::rng(); // Use a fast non-deterministic RNG
        let seed = thread_rng.random::<u64>(); // Generate a random seed
        Self { model_builder, state_builder, nruns, seed }
    }

    pub fn run_trajectory(&mut self, rng: &mut SmallRng) -> Result<(), Box<dyn Error>> {
        let model = self.model_builder.sample(false, rng).unwrap();
        let state = self.state_builder.sample(false, rng).unwrap();
        OdeProblem::new(model).solve_adaptive(
            &state,
            (0.0, 1.0),
            crate::stepping::AdaptiveStepControl::default(),
            crate::solvers::Solver::Verner9,
        )?;

        Ok(())
    }

    pub fn solve(&mut self) -> Result<(), Box<dyn Error>> {
        let mut rng = SmallRng::seed_from_u64(self.seed);
        for _ in 0..self.nruns {
            self.run_trajectory(&mut rng)?;
        }
        Ok(())
    }
}
