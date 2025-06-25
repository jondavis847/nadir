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
        ModelBuilder: Uncertainty + Clone + Send + Sync, // Need Send + Sync to share across threads
        ModelBuilder::Output: OdeModel<State = State>,
        StateBuilder: Uncertainty<Output = State> + Clone + Send + Sync, // Need Send + Sync to share
        State: OdeState + Send + Sync,
        F: Fn(
                &OdeSolver,
                OdeProblem<ModelBuilder::Output, State>,
                State,
                (f64, f64),
            ) -> Result<Option<MemoryResult<State>>, Box<dyn Error>>
            + Send
            + Sync,
    {
        // Decide if we're saving results
        let should_save = matches!(
            self.save_method,
            SaveMethods::Memory
        );

        // Create indices for parallel processing
        let indices: Vec<_> = (0..problem.nruns).collect();
        let seed = problem.seed;

        // Create a reference to the model builder and state builder for use in threads
        let model_builder = &problem.model_builder;
        let state_builder = &x0;

        // Process each run in parallel, but sampling occurs on each thread
        let results: Vec<_> = indices
            .into_par_iter()
            .map(move |i| {
                // Each thread gets its own RNG with deterministic seed based on run index
                let mut thread_rng = SmallRng::seed_from_u64(seed.wrapping_add(i as u64));

                // Clone builders and sample on this thread
                let model_builder = model_builder.clone();
                let model = model_builder
                    .sample(false, &mut thread_rng)
                    .map_err(|e| {
                        Box::<dyn Error>::from(format!(
                            "Run {}: Model sampling error: {:?}",
                            i, e
                        ))
                    })?;

                let state_builder = state_builder.clone();
                let state = state_builder
                    .sample(false, &mut thread_rng)
                    .map_err(|e| {
                        Box::<dyn Error>::from(format!(
                            "Run {}: State sampling error: {:?}",
                            i, e
                        ))
                    })?;

                // Create solver and problem on this thread
                let solver = OdeSolver::new(self.solver_method);
                let ode_problem = OdeProblem::new(model);

                // Call the provided solver function
                let result = solve_fn(
                    &solver,
                    ode_problem,
                    state,
                    tspan,
                )
                .map_err(|e| {
                    Box::<dyn Error>::from(format!(
                        "Run {}: Solver error: {:?}",
                        i, e
                    ))
                })?;

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
            // Sort the results by index
            let mut result_pairs: Vec<(usize, MemoryResult<State>)> = results
                .into_iter()
                .filter_map(|(idx, result_opt)| result_opt.map(|r| (idx, r)))
                .collect();

            // Sort by run index
            result_pairs.sort_by_key(|(idx, _)| *idx);

            // Extract just the results in correct order
            let final_results = result_pairs
                .into_iter()
                .map(|(_, r)| r)
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
        ModelBuilder: Uncertainty + Clone + Send + Sync,
        ModelBuilder::Output: OdeModel<State = State>,
        StateBuilder: Uncertainty<Output = State> + Clone + Send + Sync,
        State: OdeState + Adaptive + Send + Sync,
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
        ModelBuilder: Uncertainty + Clone + Send + Sync,
        ModelBuilder::Output: OdeModel<State = State>,
        StateBuilder: Uncertainty<Output = State> + Clone + Send + Sync,
        State: OdeState + Send + Sync,
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
