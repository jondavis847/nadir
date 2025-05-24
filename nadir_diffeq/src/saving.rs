use std::error::Error;

use crate::state::{Integrable, StateWriter, StateWriterBuilder};

/// Specifies the saving strategy to be used by the solver.
///
/// - `Memory`: Save all state data in memory for postprocessing.
/// - `File`: Write state data incrementally to a file via a `StateWriterBuilder`.
/// - `None`: Disables solver-side saving (user handles it via the model).
pub enum SaveMethod<State>
where
    State: Integrable + 'static,
{
    /// Save state data in memory using `MemoryResult`.
    Memory,
    /// Save state data to disk using a configured `StateWriterBuilder`.
    File(StateWriterBuilder<State>),
    /// Do not save solver output — user is responsible for all output handling.
    None,
}

/// Runtime storage for solver results, selected based on the `SaveMethod`.
///
/// - `Memory`: Accumulates states in RAM.
/// - `File`: Writes each step to a file.
/// - `None`: Performs no saving.
#[derive(Debug)]
pub enum ResultStorage<State>
where
    State: Integrable + 'static,
{
    /// In-memory vector storage of `(time, state)` tuples.
    Memory(MemoryResult<State>),
    /// File writer to stream output incrementally.
    File(StateWriter<State>),
    /// No output storage.
    None,
}

impl<State: Integrable> ResultStorage<State> {
    /// Save a `(time, state)` pair to the result store.
    ///
    /// No-op if storage is `None`.
    pub fn save(&mut self, t: f64, y: &State) -> Result<(), Box<dyn Error>> {
        match self {
            ResultStorage::Memory(result) => {
                result.insert(t, y);
                Ok(())
            }
            ResultStorage::File(writer) => writer.write(t, y),
            _ => Ok(()),
        }
    }

    /// Finalize and flush result storage.
    ///
    /// For `Memory`, this truncates unused buffer capacity.
    /// For `File`, this flushes the buffered writer.
    pub fn truncate(&mut self) -> Result<(), Box<dyn Error>> {
        match self {
            ResultStorage::Memory(result) => {
                result.truncate();
            }
            ResultStorage::File(writer) => writer.flush()?,
            _ => {}
        }
        Ok(())
    }
}

/// A preallocated and growable result container used for in-memory storage
/// of ODE solver outputs. Each entry stores the time and state value at that time.
#[derive(Debug)]
pub struct MemoryResult<State>
where
    State: Integrable,
{
    /// Recorded times.
    pub t: Vec<f64>,
    /// Recorded states.
    pub y: Vec<State>,
    /// Current insert index.
    i: usize,
}

impl<State: Integrable> MemoryResult<State> {
    /// Constructs a new memory result buffer with an initial capacity `n`.
    pub fn new(n: usize) -> Self {
        Self {
            t: vec![0.0; n],
            y: vec![State::default(); n],
            i: 0,
        }
    }

    /// Inserts a new result `(t, x)` into the buffer. Automatically grows if full.
    fn insert(&mut self, t: f64, x: &State) {
        if self.i == self.t.len() - 1 {
            self.extend();
        }
        self.t[self.i] = t;
        self.y[self.i].clone_from(x);
        self.i += 1;
    }

    /// Returns the total capacity of the buffer (not the number of saved entries).
    fn len(&self) -> usize {
        self.t.len()
    }

    /// Doubles the size of the buffer to accommodate more entries.
    fn extend(&mut self) {
        self.t.extend(vec![0.0; self.len()]);
        self.y.extend(vec![State::default(); self.len()]);
    }

    /// Truncates the buffer to contain only the filled entries.
    fn truncate(&mut self) {
        self.t.truncate(self.i);
        self.y.truncate(self.i);
    }
}
