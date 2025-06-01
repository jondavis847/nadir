use csv::Writer;
use std::{error::Error, path::PathBuf};

use std::fmt::{Debug, Write};
use std::fs::File;
use std::io::BufWriter;

use crate::state::OdeState;
use crate::state::state_vector::StateVector;

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
    File { root_folder: PathBuf },
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
    File { writers: Vec<StateWriter> },
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
            ResultStorage::File { writers } => {
                // model just populates the string buffers
                for writer in writers.iter_mut() {
                    y.write_vector(&mut writer.float_buffer);
                    writer.write_record(t)?;
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
    ncols: usize,
    relative_file_path: PathBuf,
    // specify specific indices of the StateVector write
    indeces: Option<Vec<usize>>,
}

impl StateWriterBuilder {
    pub fn new(ncols: usize, relative_file_path: PathBuf) -> Self {
        Self {
            relative_file_path,
            headers: None,
            ncols,
            indeces: None,
        }
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
        let headers = headers.iter().map(|header| header.to_string()).collect();
        self.headers = Some(headers);
        Ok(self)
    }

    pub fn with_indeces<const N: usize>(
        mut self,
        indeces: [usize; N],
    ) -> Result<Self, Box<dyn Error>> {
        if N > self.ncols {
            return Err(format!(
                "number of indeces ({}) must be less than or equal to ncols ({})",
                N, self.ncols
            )
            .into());
        }
        for i in indeces {
            if i >= self.ncols {
                return Err(format!(
                    "indeces must be less than or equal to ncols ({}), got ({})",
                    i, self.ncols
                )
                .into());
            }
        }
        self.indeces = Some(indeces.into());
        Ok(self)
    }
}

#[derive(Debug)]
pub struct StateWriter {
    float_buffer: StateVector,
    string_buffer: Vec<String>,
    writer: Writer<BufWriter<File>>,
    indeces: Option<Vec<usize>>,
}

impl StateWriter {
    pub fn from_builder(
        builder: &StateWriterBuilder,
        folder_path: &PathBuf,
    ) -> Result<Self, Box<dyn Error>> {
        let abs_file_path = folder_path.join(&builder.relative_file_path);
        let file = File::create(&abs_file_path)?;
        let mut writer = Writer::from_writer(BufWriter::new(file));
        let string_buffer = vec![String::new(); builder.ncols];
        let float_buffer = StateVector::with_capacity(builder.ncols);
        if let Some(headers) = &builder.headers {
            writer.write_record(headers)?;
        }
        Ok(Self {
            float_buffer,
            string_buffer,
            writer,
            indeces: builder.indeces.clone(),
        })
    }

    pub fn flush(&mut self) -> Result<(), Box<dyn Error>> {
        self.writer.flush()?;
        Ok(())
    }

    /// Writes the column data stored in the string buffers to the csv record
    pub fn write_record(&mut self, t: f64) -> Result<(), Box<dyn Error>> {
        let state = &self.float_buffer;
        write!(self.string_buffer[0], "{}", t.to_string())?;
        if let Some(indeces) = &self.indeces {
            // write only the indeces we specified
            for (i, index) in indeces.iter().enumerate() {
                write!(self.string_buffer[i + 1], "{}", state[*index].to_string())?;
            }
        } else {
            // write the whole statevector
            for i in 0..state.len() {
                write!(self.string_buffer[i], "{}", state[i].to_string())?;
            }
        }
        self.writer.write_record(&self.string_buffer)?;
        self.string_buffer.iter_mut().for_each(|e| e.clear());
        Ok(())
    }
}
