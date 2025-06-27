use indicatif::ProgressBar;
use rand::{Rng, SeedableRng, rngs::SmallRng};
use rayon::prelude::*;
use std::clone::Clone;

use std::sync::atomic::{AtomicUsize, Ordering};

use std::{error::Error, path::PathBuf};
use uncertainty::Uncertainty;

use crate::{
    OdeModel, OdeProblem,
    events::{ContinuousEvent, EventManager, PeriodicEvent, PostSimEvent, PreSimEvent, SaveEvent},
    model::{StateFromModel, StateFromModelMut},
    saving::{MemoryResult, SaveMethods},
    solvers::{OdeSolver, RungeKuttaMethods, SolverMethods},
    state::{Adaptive, OdeState},
    stepping::AdaptiveStepControl,
};

#[derive(Clone, Copy)]
pub struct MonteCarloSolver {
    save_method: SaveMethods,
    solver_method: SolverMethods,
}

impl Default for MonteCarloSolver {
    fn default() -> Self {
        Self {
            save_method: SaveMethods::Memory,
            solver_method: RungeKuttaMethods::Tsit5.into(),
        }
    }
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
    ) -> Result<Option<Vec<MemoryResult<State>>>, Box<dyn Error>>
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
                &mut ProgressBar,
            ) -> Result<Option<MemoryResult<State>>, Box<dyn Error>>
            + Send
            + Sync,
    {
        use indicatif::{MultiProgress, ProgressBar, ProgressStyle};
        use rand::SeedableRng;
        use rand::rngs::SmallRng;
        use rayon::prelude::*;
        use std::sync::{Arc, Mutex};

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

        // Create progress bars
        let bars = MultiProgress::new();

        // Add overall progress bar at the top
        let overall_pb = bars.add(ProgressBar::new(
            problem.nruns as u64,
        ));
        overall_pb.set_style(
            ProgressStyle::with_template(
                "[{elapsed_precise}] Monte Carlo: {pos}/{len} [{wide_bar:.cyan/blue}] {percent}% (ETA: {eta})",
            )
            .unwrap()
            .progress_chars("##-"),
        );
        overall_pb.set_message("Monte Carlo Simulation");

        let mut pb = Vec::new();
        let num_threads = rayon::current_num_threads();

        for _ in 0..num_threads {
            pb.push(Arc::new(Mutex::new(
                bars.add(
                    ProgressBar::new(100).with_style(
                        ProgressStyle::with_template(
                            "[{elapsed_precise}] Thread: {msg} [{wide_bar:.cyan/blue}] {percent}%",
                        )
                        .unwrap()
                        .progress_chars("##-"),
                    ),
                ),
            )));
        }

        let bars_in_use = Arc::new(Mutex::new(vec![
            false;
            num_threads
        ]));
        let completed_runs = Arc::new(AtomicUsize::new(0));

        // Process each run in parallel
        let results: Vec<_> = indices
            .into_par_iter()
            .map(
                |i| -> Result<
                    (
                        usize,
                        Option<MemoryResult<State>>,
                    ),
                    MonteCarloError,
                > {
                    // Acquire a progress bar
                    let bar_index = {
                        let mut bars_in_use = bars_in_use
                            .lock()
                            .unwrap();
                        let mut found_index = None;
                        for (idx, in_use) in bars_in_use
                            .iter_mut()
                            .enumerate()
                        {
                            if !*in_use {
                                *in_use = true;
                                found_index = Some(idx);
                                break;
                            }
                        }
                        found_index.expect("No progress bar available - this should not happen!")
                    };

                    // Use the progress bar
                    let progress_bar = pb[bar_index].clone();
                    let progress_bar = &mut *progress_bar
                        .lock()
                        .unwrap();
                    {
                        progress_bar.reset();
                        progress_bar.set_message(format!(
                            "{} Run: {}",
                            bar_index,
                            i + 1
                        ));
                        progress_bar.set_length(100); // You may want to adjust this based on your actual work
                    }

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
                    let mut ode_problem = OdeProblem::new(model).with_events(
                        problem
                            .events
                            .clone(),
                    );

                    // If there are saving events, update the result folder path with the run number
                    if let Some(save_folder) = &problem.save_folder {
                        ode_problem =
                            ode_problem.with_saving(save_folder.join(format!("run{}", i)));
                    }

                    // Call the provided solver function
                    let result = solve_fn(
                        &solver,
                        ode_problem,
                        state,
                        tspan,
                        progress_bar,
                    )
                    .map_err(|e| {
                        Box::<dyn Error>::from(format!(
                            "Run {}: Solver error: {:?}",
                            i, e
                        ))
                    })?;

                    // Release the progress bar and mark as completed
                    progress_bar.finish();

                    let completed = completed_runs.fetch_add(1, Ordering::Relaxed) + 1;
                    overall_pb.set_position(completed as u64);

                    // Release the progress bar for next use
                    {
                        let mut bars_in_use = bars_in_use
                            .lock()
                            .unwrap();
                        bars_in_use[bar_index] = false;
                    }

                    // Only return results if we need to save them
                    Ok((
                        i,
                        if should_save {
                            result
                        } else {
                            None
                        },
                    ))
                },
            )
            .collect::<Result<Vec<_>, MonteCarloError>>()?;

        overall_pb.finish_with_message("Monte Carlo simulation completed!");

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

    // Private helper method that handles the common logic
    fn solve_model_monte_carlo<ModelBuilder, State, F>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        tspan: (f64, f64),
        solve_fn: F,
    ) -> Result<Option<Vec<MemoryResult<State>>>, Box<dyn Error>>
    where
        ModelBuilder: Uncertainty + Clone + Send + Sync, // Need Send + Sync to share across threads
        ModelBuilder::Output: OdeModel<State = State> + StateFromModel<State = State>,
        State: OdeState + Send + Sync,
        F: Fn(
                &OdeSolver,
                OdeProblem<ModelBuilder::Output, State>,
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

                // Create solver and problem on this thread
                let solver = OdeSolver::new(self.solver_method);
                let mut ode_problem = OdeProblem::new(model).with_events(
                    problem
                        .events
                        .clone(),
                );
                // If there are saving events, update the result folder path with the run number
                if let Some(save_folder) = &problem.save_folder {
                    ode_problem = ode_problem.with_saving(save_folder.join(format!("run{i}")));
                }

                // Call the provided solver function
                let result = solve_fn(&solver, ode_problem, tspan).map_err(|e| {
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

    // Private helper method that handles the common logic
    fn solve_model_monte_carlo_mut<ModelBuilder, State, F>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        tspan: (f64, f64),
        solve_fn: F,
    ) -> Result<Option<Vec<MemoryResult<State>>>, Box<dyn Error>>
    where
        ModelBuilder: Uncertainty + Clone + Send + Sync, // Need Send + Sync to share across threads
        ModelBuilder::Output: OdeModel<State = State> + StateFromModelMut,
        State: OdeState + Send + Sync,
        F: Fn(
                &OdeSolver,
                OdeProblem<ModelBuilder::Output, State>,
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

                // Create solver and problem on this thread
                let solver = OdeSolver::new(self.solver_method);
                let mut ode_problem = OdeProblem::new(model).with_events(
                    problem
                        .events
                        .clone(),
                );
                // If there are saving events, update the result folder path with the run number
                if let Some(save_folder) = &problem.save_folder {
                    ode_problem = ode_problem.with_saving(save_folder.join(format!("run{i}")));
                }

                // Call the provided solver function
                let result = solve_fn(&solver, ode_problem, tspan).map_err(|e| {
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
    ) -> Result<Option<Vec<MemoryResult<State>>>, Box<dyn Error>>
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
            move |solver, problem, state, tspan, progress_bar| {
                solver.solve_adaptive_progress(
                    problem,
                    state,
                    tspan,
                    controller,
                    progress_bar,
                )
            },
        )
    }

    pub fn solve_model_adaptive<ModelBuilder, State>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        tspan: (f64, f64),
        controller: AdaptiveStepControl,
    ) -> Result<Option<Vec<MemoryResult<State>>>, Box<dyn Error>>
    where
        ModelBuilder: Uncertainty + Clone + Send + Sync,
        ModelBuilder::Output: OdeModel<State = State> + StateFromModel<State = State>,
        State: OdeState + Adaptive + Send + Sync,
    {
        self.solve_model_monte_carlo(
            problem,
            tspan,
            move |solver, problem, tspan| {
                let x0 = problem
                    .model
                    .initial_state();
                solver.solve_adaptive(problem, x0, tspan, controller)
            },
        )
    }

    pub fn solve_model_adaptive_mut<ModelBuilder, State>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        tspan: (f64, f64),
        controller: AdaptiveStepControl,
    ) -> Result<Option<Vec<MemoryResult<State>>>, Box<dyn Error>>
    where
        ModelBuilder: Uncertainty + Clone + Send + Sync,
        ModelBuilder::Output: OdeModel<State = State> + StateFromModelMut<State = State>,
        State: OdeState + Adaptive + Send + Sync,
    {
        self.solve_model_monte_carlo_mut(
            problem,
            tspan,
            move |solver, mut problem, tspan| {
                let x0 = problem
                    .model
                    .initial_state();
                solver.solve_adaptive(problem, x0, tspan, controller)
            },
        )
    }

    pub fn solve_fixed<ModelBuilder, StateBuilder, State>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        x0: StateBuilder,
        tspan: (f64, f64),
        dt: f64,
    ) -> Result<Option<Vec<MemoryResult<State>>>, Box<dyn Error>>
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
            move |solver, problem, state, tspan, progress_bar| {
                solver.solve_fixed_progress(
                    problem,
                    state,
                    tspan,
                    dt,
                    progress_bar,
                )
            },
        )
    }

    pub fn solve_model_fixed<ModelBuilder, State>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        tspan: (f64, f64),
        dt: f64,
    ) -> Result<Option<Vec<MemoryResult<State>>>, Box<dyn Error>>
    where
        ModelBuilder: Uncertainty + Clone + Send + Sync,
        ModelBuilder::Output: OdeModel<State = State> + StateFromModel<State = State>,
        State: OdeState + Send + Sync,
    {
        self.solve_model_monte_carlo(
            problem,
            tspan,
            move |solver, problem, tspan| {
                let x0 = problem
                    .model
                    .initial_state();
                solver.solve_fixed(problem, x0, tspan, dt)
            },
        )
    }

    pub fn solve_model_fixed_mut<ModelBuilder, State>(
        &self,
        problem: MonteCarloProblem<ModelBuilder>,
        tspan: (f64, f64),
        dt: f64,
    ) -> Result<Option<Vec<MemoryResult<State>>>, Box<dyn Error>>
    where
        ModelBuilder: Uncertainty + Clone + Send + Sync,
        ModelBuilder::Output: OdeModel<State = State> + StateFromModelMut<State = State>,
        State: OdeState + Send + Sync,
    {
        self.solve_model_monte_carlo_mut(
            problem,
            tspan,
            move |solver, mut problem, tspan| {
                let x0 = problem
                    .model
                    .initial_state();
                solver.solve_fixed(problem, x0, tspan, dt)
            },
        )
    }
}

pub struct MonteCarloProblem<ModelBuilder>
where
    ModelBuilder: Uncertainty,
    ModelBuilder::Output: OdeModel,
{
    events: EventManager<ModelBuilder::Output, <ModelBuilder::Output as OdeModel>::State>,
    model_builder: ModelBuilder,
    nruns: usize,
    save_folder: Option<PathBuf>,
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
        Self {
            model_builder,
            nruns,
            seed,
            save_folder: None,
            events: EventManager::new(),
        }
    }

    /// Adds a periodic event to the simulation.
    pub fn with_periodic_event(
        mut self,
        event: PeriodicEvent<ModelBuilder::Output, <ModelBuilder::Output as OdeModel>::State>,
    ) -> Self {
        self.events
            .add_periodic(event);
        self
    }

    // Adds a presim event to the simulation.
    pub fn with_presim_event(
        mut self,
        event: PreSimEvent<ModelBuilder::Output, <ModelBuilder::Output as OdeModel>::State>,
    ) -> Self {
        self.events
            .add_presim(event);
        self
    }

    /// Adds a postsim event to the simulation.
    pub fn with_postsim_event(mut self, event: PostSimEvent<ModelBuilder::Output>) -> Self {
        self.events
            .add_postsim(event);
        self
    }

    /// Adds a continuous event to the simulation.
    pub fn with_continuous_event(
        mut self,
        event: ContinuousEvent<ModelBuilder::Output, <ModelBuilder::Output as OdeModel>::State>,
    ) -> Self {
        self.events
            .add_continuous(event);
        self
    }

    /// Adds the ability to save results to a folder on disk
    pub fn with_saving(mut self, save_folder: PathBuf) -> Self {
        self.save_folder = Some(save_folder);
        self
    }

    /// Adds a continuous event to the simulation.
    pub fn with_save_event(
        mut self,
        event: SaveEvent<ModelBuilder::Output, <ModelBuilder::Output as OdeModel>::State>,
    ) -> Self {
        if self
            .save_folder
            .is_some()
        {
            self.events
                .add_save(event);
            self
        } else {
            panic!("need to call with_saving() before with_save_event() to enable saving")
        }
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
