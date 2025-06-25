use rand::{Rng, SeedableRng, rngs::SmallRng};
use rayon::prelude::*;
use std::error::Error;
use uncertainty::Uncertainty;

use crate::{
    OdeModel, OdeProblem,
    saving::{MemoryResult, SaveMethods},
    solvers::{OdeSolver, SolverMethods},
    state::{Adaptive, OdeState},
    stepping::AdaptiveStepControl,
};

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

    // Private helper method that handles the common logic
    fn solve_monte_carlo<ModelBuilder, StateBuilder, State, F>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        x0: StateBuilder,
        tspan: (f64, f64),
        solve_fn: F,
    ) -> Result<Option<Vec<MemoryResult<StateBuilder::Output>>>, Box<dyn Error>>
    where
        ModelBuilder: Uncertainty,
        ModelBuilder::Output: OdeModel<State = State>,
        StateBuilder: Uncertainty<Output = State>,
        State: OdeState,
        F: Fn(
                &OdeSolver,
                OdeProblem<ModelBuilder::Output, State>,
                State,
                (f64, f64),
            ) -> Result<Option<MemoryResult<State>>, Box<dyn Error>>
            + Send
            + Sync,
    {
        // Create our RNG with the specified seed
        let mut rng = SmallRng::seed_from_u64(problem.seed);

        // Sample all models and states sequentially
        let mut models = Vec::with_capacity(problem.nruns);
        let mut states = Vec::with_capacity(problem.nruns);

        for _ in 0..problem.nruns {
            // Sample model
            let model = problem
                .model_builder
                .sample(false, &mut rng)
                .map_err(|e| {
                    Box::<dyn Error>::from(format!(
                        "Model sampling error: {:?}",
                        e
                    ))
                })?;
            models.push(model);

            // Sample initial state
            let state = x0
                .sample(false, &mut rng)
                .map_err(|e| {
                    Box::<dyn Error>::from(format!(
                        "State sampling error: {:?}",
                        e
                    ))
                })?;
            states.push(state);
        }

        // Decide if we're saving results
        let should_save = matches!(
            self.save_method,
            SaveMethods::Memory
        );

        // Solve each problem in parallel
        let results: Vec<_> = models
            .into_iter()
            .zip(states)
            .enumerate()
            .par_bridge() // Convert to parallel iterator
            .map(|(i, (model, state))| {
                let solver = OdeSolver::new(self.solver_method);
                let problem = OdeProblem::new(model);

                // Call the provided solver function
                let result = solve_fn(&solver, problem, state, tspan)?;

                // Only return results if we need to save them
                Ok((
                    i,
                    if should_save {
                        result
                    } else {
                        None
                    },
                ))
            })
            .collect::<Result<Vec<_>, MonteCarloError>>()?;

        // If we're saving results, assemble them in the correct order
        if should_save {
            // Initialize an empty vector with the right capacity
            let mut ordered_results = Vec::with_capacity(problem.nruns);

            // Pre-fill with None values
            for _ in 0..problem.nruns {
                ordered_results.push(None);
            }

            // Insert each result at its original index position
            for (idx, result) in results {
                ordered_results[idx] = result;
            }

            // Filter out None values
            let final_results = ordered_results
                .into_iter()
                .flatten()
                .collect();
            Ok(Some(final_results))
        } else {
            Ok(None)
        }
    }

    pub fn solve_adaptive<ModelBuilder, StateBuilder, State>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        x0: StateBuilder,
        tspan: (f64, f64),
        controller: AdaptiveStepControl,
    ) -> Result<Option<Vec<MemoryResult<StateBuilder::Output>>>, Box<dyn Error>>
    where
        ModelBuilder: Uncertainty,
        ModelBuilder::Output: OdeModel<State = State>,
        StateBuilder: Uncertainty<Output = State>,
        State: OdeState + Adaptive,
    {
        self.solve_monte_carlo(
            problem,
            x0,
            tspan,
            move |solver, problem, state, tspan| {
                solver.solve_adaptive(
                    problem, state, tspan, controller,
                )
            },
        )
    }

    pub fn solve_fixed<ModelBuilder, StateBuilder, State>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        x0: StateBuilder,
        tspan: (f64, f64),
        dt: f64,
    ) -> Result<Option<Vec<MemoryResult<StateBuilder::Output>>>, Box<dyn Error>>
    where
        ModelBuilder: Uncertainty,
        ModelBuilder::Output: OdeModel<State = State>,
        StateBuilder: Uncertainty<Output = State>,
        State: OdeState,
    {
        self.solve_monte_carlo(
            problem,
            x0,
            tspan,
            move |solver, problem, state, tspan| solver.solve_fixed(problem, state, tspan, dt),
        )
    }
}

pub struct MonteCarloProblem<ModelBuilder>
where
    ModelBuilder: Uncertainty,
    ModelBuilder::Output: OdeModel,
{
    model_builder: ModelBuilder,
    nruns: usize,
    seed: u64,
}

impl<ModelBuilder> MonteCarloProblem<ModelBuilder>
where
    ModelBuilder: Uncertainty,
    ModelBuilder::Output: OdeModel,
{
    /// Creates a new `MonteCarloProblem` instance with the specified configuration.
    pub fn new(model_builder: ModelBuilder, nruns: usize) -> Self {
        let mut thread_rng = rand::rng(); // Use a fast non-deterministic RNG
        let seed = thread_rng.random::<u64>(); // Generate a random seed
        Self { model_builder, nruns, seed }
    }
}

// Box<dyn Error + Send + Sized + Sync> use auto-traits which apparently cant cross boundaries or something
// so we need a custom error type
#[derive(Debug)]
pub struct MonteCarloError {
    message: String,
}

impl MonteCarloError {
    pub fn new<S: Into<String>>(message: S) -> Self {
        Self { message: message.into() }
    }
}

impl std::fmt::Display for MonteCarloError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.message)
    }
}

impl std::error::Error for MonteCarloError {}

// This is automatically Send + Sync because String is Send + Sync

// More specific implementation targeting just Box<dyn Error>
impl From<Box<dyn std::error::Error>> for MonteCarloError {
    fn from(error: Box<dyn std::error::Error>) -> Self {
        MonteCarloError::new(format!("{}", error))
    }
}
