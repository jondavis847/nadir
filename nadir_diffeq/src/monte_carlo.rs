use rand::{Rng, SeedableRng, rngs::SmallRng};
use std::error::Error;
use uncertainty::Uncertainty;

use crate::{
    OdeModel, OdeProblem,
    saving::{ResultStorage, SaveMethods},
    solvers::{OdeSolver, SolverMethods},
    state::{Adaptive, OdeState},
    stepping::AdaptiveStepControl,
};

pub trait UncertainModel: Uncertainty<Output = Self::Model> {
    type Model: OdeModel;
}

pub trait UncertainState: Uncertainty<Output = Self::State> {
    type State: OdeState;
}

#[derive(Clone, Copy)]
pub struct MonteCarloSolver {
    save_method: SaveMethods,
    solver_method: SolverMethods,
}

impl MonteCarloSolver {
    pub fn new(solver_method: SolverMethods) -> Self {
        Self { solver_method, save_method: SaveMethods::Memory }
    }

    pub fn with_save_method(mut self, save_method: SaveMethods) -> Self {
        self.save_method = save_method;
        self
    }

    pub fn solve_adaptive<ModelBuilder, StateBuilder>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        x0: StateBuilder,
        tspan: (f64, f64),
        controller: AdaptiveStepControl,
    ) -> Result<Vec<ResultStorage<StateBuilder::Output>>, Box<dyn Error>>
    where
        ModelBuilder: UncertainModel,
        StateBuilder:
            UncertainState<State = <<ModelBuilder as UncertainModel>::Model as OdeModel>::State>,
        StateBuilder::State: Adaptive,
    {
        let mut rng = SmallRng::seed_from_u64(problem.seed);
        let mut results = Vec::with_capacity(problem.nruns);
        for i in 0..problem.nruns {
            let solver = OdeSolver::new(self.solver_method);
            let model = problem
                .model_builder
                .sample(false, &mut rng)
                .unwrap();
            let state = x0.sample(false, &mut rng).unwrap();
            let problem = OdeProblem::new(model);
            results[i] = solver.solve_adaptive(problem, state, tspan, controller)?;
        }
        Ok(results)
    }

    pub fn solve_fixed<ModelBuilder, StateBuilder>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        x0: StateBuilder,
        tspan: (f64, f64),
        dt: f64,
    ) -> Result<Vec<ResultStorage<StateBuilder::Output>>, Box<dyn Error>>
    where
        ModelBuilder: UncertainModel,
        StateBuilder:
            UncertainState<State = <<ModelBuilder as UncertainModel>::Model as OdeModel>::State>,
    {
        let mut rng = SmallRng::seed_from_u64(problem.seed);
        let mut results = Vec::with_capacity(problem.nruns);
        for i in 0..problem.nruns {
            let solver = OdeSolver::new(self.solver_method);
            let model = problem
                .model_builder
                .sample(false, &mut rng)
                .unwrap();
            let state = x0.sample(false, &mut rng).unwrap();
            let problem = OdeProblem::new(model);
            results[i] = solver.solve_fixed(problem, state, tspan, dt)?;
        }
        Ok(results)
    }
}

pub struct MonteCarloProblem<ModelBuilder>
where
    ModelBuilder: UncertainModel,
{
    model_builder: ModelBuilder,
    nruns: usize,
    seed: u64,
}

impl<ModelBuilder> MonteCarloProblem<ModelBuilder>
where
    ModelBuilder: UncertainModel,
{
    /// Creates a new `MonteCarloProblem` instance with the specified configuration.
    pub fn new(model_builder: ModelBuilder, nruns: usize) -> Self {
        let mut thread_rng = rand::rng(); // Use a fast non-deterministic RNG
        let seed = thread_rng.random::<u64>(); // Generate a random seed
        Self { model_builder, nruns, seed }
    }
}
