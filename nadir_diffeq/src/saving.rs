use csv::Writer;
use std::collections::HashMap;
use std::{error::Error, path::PathBuf};

use std::fmt::{Debug, Write};
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
    pub fn save<M: OdeModel<State = S>>(
        &mut self,
        model: &M,
        t: f64,
        y: &S,
    ) -> Result<(), Box<dyn Error>> {
        match self {
            ResultStorage::Memory(result) => {
                result.insert(t, y);
                Ok(())
            }
            ResultStorage::File(manager) => {
                // model just populates the string buffers
                model.write_record(t, y, manager)?;

                // now write the buffers to the files
                for writer in manager.writers.values_mut() {
                    writer.write_record()?;
                }
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
pub struct MemoryResult<S>
where
    S: State,
{
    /// Recorded times.
    pub t: Vec<f64>,
    /// Recorded states.
    pub y: Vec<S>,
    /// Current insert index.
    i: usize,
}

impl<S> MemoryResult<S>
where
    S: State,
{
    /// Constructs a new memory result buffer with an initial capacity `n`.
    pub fn new(n: usize) -> Self {
        Self {
            t: vec![0.0; n],
            y: vec![S::default(); n],
            i: 0,
        }
    }

    /// Inserts a new result `(t, x)` into the buffer. Automatically grows if full.
    fn insert(&mut self, t: f64, x: &S) {
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
        self.y.extend(vec![S::default(); self.len()]);
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
    headers: Vec<String>,
    ncols: usize,
}

impl StateWriterBuilder {
    pub fn new<S: State>(file_path: PathBuf, ncols: usize) -> Self {
        let headers = S::headers(ncols);
        if headers.len() != ncols {
            panic!(
                "length of headers ({}) doesn't match writer ncols ({})",
                headers.len(),
                ncols
            );
        }
        Self {
            file_path,
            headers,
            ncols,
        }
    }
}

#[derive(Debug)]
pub struct StateWriter {
    ncols: usize,
    buffer: Vec<String>,
    writer: Writer<BufWriter<File>>,
}

impl TryFrom<&StateWriterBuilder> for StateWriter {
    type Error = Box<dyn Error>;

    fn try_from(value: &StateWriterBuilder) -> Result<Self, Self::Error> {
        let file = File::create(&value.file_path)?;
        let mut writer = Writer::from_writer(BufWriter::new(file));
        let buffer = vec![String::new(); value.ncols];
        if !value.headers.is_empty() {
            writer.write_record(&value.headers)?;
        }
        Ok(Self {
            buffer,
            writer,
            ncols: value.ncols,
        })
    }
}

impl StateWriter {
    pub fn flush(&mut self) -> Result<(), Box<dyn Error>> {
        self.writer.flush()?;
        Ok(())
    }

    /// Helper function to write f64 data to a column
    pub fn write_column(&mut self, column: usize, value: f64) -> Result<(), Box<dyn Error>> {
        if column >= self.ncols {
            panic!("Column index out of bounds: {} >= {}", column, self.ncols);
        }
        write!(self.buffer[column], "{},", value)?;
        Ok(())
    }

    /// Writes the column data stored in the string buffers to the csv record
    pub fn write_record(&mut self) -> Result<(), Box<dyn Error>> {
        self.writer.write_record(&self.buffer)?;
        for i in 0..self.ncols {
            self.buffer[i].clear(); // Clear the buffer for the next record
        }
        Ok(())
    }
}

#[derive(Debug)]
pub struct WriterManagerBuilder {
    pub dir_path: PathBuf,
    counter: u32,
    writers: HashMap<WriterId, StateWriterBuilder>,
}

impl WriterManagerBuilder {
    pub fn add_writer<S: State>(
        &mut self,
        file_path: PathBuf,
        ncols: usize,
    ) -> Result<WriterId, Box<dyn Error>> {
        self.counter += 1;
        let id = WriterId(self.counter);
        let writer = StateWriterBuilder::new::<S>(self.dir_path.join(file_path), ncols);
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

#[derive(Debug, Copy, Clone, Hash, Eq, PartialEq, Default)]
pub struct WriterId(u32);

impl WriterManager {
    pub fn get_writer(&mut self, id: &WriterId) -> &mut StateWriter {
        if let Some(writer) = self.writers.get_mut(id) {
            writer
        } else {
            panic!("Writer with ID {:?} not found", id);
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
        std::fs::create_dir_all(&value.dir_path)?;
        for (id, builder) in &value.writers {
            writers.insert(*id, StateWriter::try_from(builder)?);
        }
        Ok(Self { writers })
    }
}
