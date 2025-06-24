use csv::Writer;
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::{error::Error, path::PathBuf};

use std::fmt::{Debug, Write};
use std::fs::{File, create_dir_all};
use std::io::BufWriter;

use crate::state::OdeState;

/// Specifies the saving strategy to be used by the solver.
///
/// - `Memory`: Save all state data in memory for postprocessing.
/// - `File`: Write state data incrementally to a file via a `StateWriterBuilder`.
/// - `None`: Disables solver-side saving (user handles it via the model).
#[derive(Clone, Copy)]
pub enum SaveMethods {
    /// Save state data in memory using `MemoryResult`.
    Memory,
    /// Save state data to disk using a configured `StateWriterBuilder`.
    /// Each element of the vector is a separate file.
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
    State: OdeState,
{
    /// In-memory vector storage of `(time, state)` tuples.
    Memory(MemoryResult<State>),
    /// File writer to stream output incrementally.
    File {
        writers: Vec<StateWriter>,
    },
    /// No output storage.
    None,
}

impl<State> ResultStorage<State>
where
    State: OdeState,
{
    /// Save a `(time, state)` pair to the result store.
    ///
    /// No-op if storage is `None`.
    pub fn save(&mut self, t: f64, y: &State) -> Result<(), Box<dyn Error>> {
        match self {
            ResultStorage::Memory(result) => {
                result.insert(t, y);
                Ok(())
            }
            // ResultStorage::File { writers } => {
            //     // model just populates the string buffers
            //     for writer in writers.iter_mut() {
            //         y.write_vector(&mut writer.float_buffer);
            //         writer.write_record(t)?;
            //     }

            //     Ok(())
            // }
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
            ResultStorage::File { writers } => {
                for writer in writers {
                    writer.flush()?;
                }
            }
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
    State: OdeState,
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
    State: OdeState,
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

#[derive(Debug, Clone)]
pub struct StateWriterBuilder {
    headers: Option<Vec<String>>,
    pub ncols: usize,
    relative_file_path: PathBuf,
}

impl StateWriterBuilder {
    pub fn new(ncols: usize, relative_file_path: PathBuf) -> Self {
        Self { relative_file_path, headers: None, ncols }
    }

    pub fn with_headers(mut self, headers: &[&str]) -> Result<Self, Box<dyn Error>> {
        if headers.len() != self.ncols {
            return Err(format!(
                "header length ({}) must be equal to ncols ({})",
                headers.len(),
                self.ncols
            )
            .into());
        }
        let headers = headers
            .iter()
            .map(|header| header.to_string())
            .collect();
        self.headers = Some(headers);
        Ok(self)
    }
}

#[derive(Debug)]
pub struct StateWriter {
    pub float_buffer: Vec<f64>,
    string_buffer: Vec<String>,
    writer: Writer<BufWriter<File>>,
}

impl StateWriter {
    pub fn from_builder(
        builder: &StateWriterBuilder,
        folder_path: &PathBuf,
    ) -> Result<Self, Box<dyn Error>> {
        let abs_file_path = folder_path.join(&builder.relative_file_path);
        if let Some(parent) = abs_file_path.parent() {
            create_dir_all(parent)?;
        }
        let file = File::create(&abs_file_path)?;
        let mut writer = Writer::from_writer(BufWriter::new(file));
        let string_buffer = vec![String::new(); builder.ncols];
        let float_buffer = vec![0.0; builder.ncols];
        if let Some(headers) = &builder.headers {
            writer.write_record(headers)?;
        }
        Ok(Self { float_buffer, string_buffer, writer })
    }

    pub fn flush(&mut self) -> Result<(), Box<dyn Error>> {
        self.writer.flush()?;
        Ok(())
    }

    /// Writes the column data stored in the string buffers to the csv record
    pub fn write_record(&mut self) -> Result<(), Box<dyn Error>> {
        self.string_buffer
            .iter_mut()
            .for_each(|e| e.clear());
        let state = &self.float_buffer;

        // write the whole statevector
        for i in 0..state.len() {
            write!(self.string_buffer[i], "{}", state[i].to_string())?;
        }

        self.writer.write_record(&self.string_buffer)?;
        Ok(())
    }
}

pub struct WriterManager {
    pub root_dir: Option<PathBuf>, // set when we initialize
    id_ctr: u32,
    builders: HashMap<WriterId, StateWriterBuilder>,
    pub writers: HashMap<WriterId, StateWriter>,
}

impl WriterManager {
    pub fn new() -> Self {
        Self {
            root_dir: None,
            id_ctr: 0,
            builders: HashMap::new(),
            writers: HashMap::new(),
        }
    }

    pub fn add_writer(&mut self, writer: StateWriterBuilder) -> WriterId {
        self.id_ctr += 1;
        self.builders
            .insert(WriterId(self.id_ctr), writer);
        WriterId(self.id_ctr)
    }

    pub fn initialize(&mut self, root_path: &PathBuf) -> Result<(), Box<dyn Error>> {
        self.root_dir = Some(root_path.clone());
        for (id, builder) in &self.builders {
            let writer = StateWriter::from_builder(builder, root_path)?;
            self.writers.insert(*id, writer);
        }
        Ok(())
    }
}

#[derive(Debug, Copy, Clone, Eq, Hash, PartialEq, Deserialize, Serialize)]
pub struct WriterId(u32);
