use csv::Writer;
use std::collections::HashMap;
use std::{error::Error, path::PathBuf};

use std::fmt::Debug;
use std::fs::File;
use std::io::BufWriter;

use crate::OdeModel;
use crate::state::State;

/// Specifies the saving strategy to be used by the solver.
///
/// - `Memory`: Save all state data in memory for postprocessing.
/// - `File`: Write state data incrementally to a file via a `StateWriterBuilder`.
/// - `None`: Disables solver-side saving (user handles it via the model).
pub enum SaveMethod {
    /// Save state data in memory using `MemoryResult`.
    Memory,
    /// Save state data to disk using a configured `StateWriterBuilder`.
    /// Each element of the vector is a separate file.
    File(WriterManagerBuilder),
    /// Do not save solver output — user is responsible for all output handling.
    None,
}

/// Runtime storage for solver results, selected based on the `SaveMethod`.
///
/// - `Memory`: Accumulates states in RAM.
/// - `File`: Writes each step to a file.
/// - `None`: Performs no saving.
#[derive(Debug)]
pub enum ResultStorage<S>
where
    S: State,
{
    /// In-memory vector storage of `(time, state)` tuples.
    Memory(MemoryResult<S>),
    /// File writer to stream output incrementally.
    File(WriterManager),
    /// No output storage.
    None,
}

impl<S> ResultStorage<S>
where
    S: State,
{
    /// Save a `(time, state)` pair to the result store.
    ///
    /// No-op if storage is `None`.
    pub fn save<Model: OdeModel<State = S>>(
        &mut self,
        model: &Model,
        t: f64,
        y: &S,
    ) -> Result<(), Box<dyn Error>> {
        match self {
            ResultStorage::Memory(result) => {
                result.insert(t, y);
                Ok(())
            }
            ResultStorage::File(manager) => {
                model.write_record(t, y, manager);
                Ok(())
            }
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
            ResultStorage::File(manager) => manager.flush()?,
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
    State: Default,
{
    /// Recorded times.
    pub t: Vec<f64>,
    /// Recorded states.
    pub y: Vec<State>,
    /// Current insert index.
    i: usize,
}

impl<State> MemoryResult<State>
where
    State: WritableState,
{
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

#[derive(Debug)]
pub struct StateWriterBuilder {
    file_path: PathBuf,
}

impl StateWriterBuilder {
    pub fn new(file_path: PathBuf) -> Self {
        Self { file_path }
    }
}

#[derive(Debug)]
pub struct StateWriter {
    buffer: Vec<String>,
    writer: Writer<BufWriter<File>>,
}

impl TryFrom<&StateWriterBuilder> for StateWriter {
    type Error = Box<dyn Error>;

    fn try_from(value: &StateWriterBuilder) -> Result<Self, Self::Error> {
        let file = File::create(&value.file_path)?;
        let writer = Writer::from_writer(BufWriter::new(file));
        Ok(Self {
            buffer: Vec::new(),
            writer,
        })
    }
}

impl StateWriter {
    pub fn flush(&mut self) -> Result<(), Box<dyn Error>> {
        self.writer.flush()?;
        Ok(())
    }
}
pub trait WritableState: Default + Clone {
    fn write_headers(&self, _manager: &mut Vec<String>) -> Result<(), Box<dyn Error>> {
        Ok(())
    }
    fn write_record(&self, _t: f64, _manager: &mut Vec<String>) -> Result<(), Box<dyn Error>> {
        Ok(())
    }
}

pub trait WritableModel {
    type State: State;
    fn init_writers(&mut self, _manager: &mut WriterManager) -> Result<(), Box<dyn Error>>;
    fn write_record(&self, _t: f64, _state: &Self::State, _manager: &mut WriterManager);
}

#[derive(Debug)]
pub struct WriterManagerBuilder {
    dir_path: PathBuf,
    counter: u32,
    writers: HashMap<WriterId, StateWriterBuilder>,
}

impl WriterManagerBuilder {
    pub fn add_writer(&mut self, file_path: PathBuf) -> Result<WriterId, Box<dyn Error>> {
        self.counter += 1;
        let id = WriterId(self.counter);
        let writer = StateWriterBuilder::new(self.dir_path.join(file_path));
        self.writers.insert(id, writer);
        Ok(id)
    }

    pub fn new(dir_path: PathBuf) -> Self {
        Self {
            dir_path,
            counter: 0,
            writers: HashMap::new(),
        }
    }
}

// Manager for multiple writers
#[derive(Debug)]
pub struct WriterManager {
    writers: HashMap<WriterId, StateWriter>,
}

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq)]
pub struct WriterId(u32);

impl WriterManager {
    pub fn get_buffer(&mut self, id: &WriterId) -> Option<&mut Vec<String>> {
        if let Some(writer) = self.writers.get_mut(id) {
            Some(&mut writer.buffer)
        } else {
            None
        }
    }

    // Flush all writers
    pub fn flush(&mut self) -> Result<(), Box<dyn Error>> {
        for (_, writer) in &mut self.writers {
            writer.flush()?;
        }
        Ok(())
    }
}

impl TryFrom<&WriterManagerBuilder> for WriterManager {
    type Error = Box<dyn Error>;
    fn try_from(value: &WriterManagerBuilder) -> Result<Self, Self::Error> {
        let mut writers = HashMap::new();
        for (id, builder) in &value.writers {
            writers.insert(*id, StateWriter::try_from(builder)?);
        }
        Ok(Self { writers })
    }
}
