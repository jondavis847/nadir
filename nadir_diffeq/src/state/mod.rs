//! A generic CSV state writer for types implementing the `Integrable` trait.
//! This module allows defining how a state is formatted into a CSV-compatible
//! row and then writing it out incrementally.

use std::{
    error::Error,
    fmt::{self, Debug},
    fs::File,
    io::BufWriter,
    marker::PhantomData,
    ops::{AddAssign, MulAssign},
    path::PathBuf,
};

use csv::Writer;
use tolerance::Tolerance;

pub mod state_array;
pub mod state_vector;

/// Trait representing an integrable state for use in ODE solvers.
///
/// Types implementing this trait must support arithmetic operations,
/// cloning, and formatting for debugging. The associated `Derivative`
/// type represents the derivative of the state, and `Tolerance` is used
/// for controlling adaptive solver behavior.
pub trait Integrable: Sized + Clone + Default + MulAssign<f64> + Debug
where
    for<'a> Self: AddAssign<&'a Self> + AddAssign<&'a Self::Derivative>,
{
    /// The derivative of the state, used in ODE computation.
    type Derivative: Clone + MulAssign<f64> + Sized + Default + Debug;

    /// The tolerance model associated with the state, used for error estimation.
    type Tolerance: Tolerance<State = Self>;

    /// Create a `StateWriterBuilder` for writing this type of state to a file.
    fn writer(path: PathBuf) -> StateWriterBuilder<Self>;
}

/// Builder for a `StateWriter`, allowing the configuration of output path,
/// headers, and formatting behavior for a state type implementing `Integrable`.
pub struct StateWriterBuilder<State>
where
    State: Integrable,
{
    path: PathBuf,
    headers: Vec<String>,
    formatter: Box<dyn StateFormatter<State>>,
    _phantom_data: PhantomData<State>,
}

impl<State> StateWriterBuilder<State>
where
    State: Integrable + 'static,
{
    /// Create a new `StateWriterBuilder` with the given file path and state formatter.
    ///
    /// # Arguments
    ///
    /// * `path` - The path to the output CSV file.
    /// * `formatter` - A formatter that determines how states are written.
    pub fn new<F>(path: PathBuf, formatter: F) -> Self
    where
        F: StateFormatter<State> + 'static,
    {
        Self {
            path,
            headers: Vec::new(),
            formatter: Box::new(formatter),
            _phantom_data: PhantomData,
        }
    }

    /// Optionally add CSV column headers.
    ///
    /// # Arguments
    ///
    /// * `headers` - An array of string slices to use as the CSV headers.
    pub fn with_headers<const N: usize>(
        mut self,
        headers: [&str; N],
    ) -> Result<Self, Box<dyn Error>> {
        let headers: Vec<String> = headers.iter().map(|h| h.to_string()).collect();
        self.headers = headers;
        Ok(self)
    }

    /// Create the actual `StateWriter` from the builder.
    ///
    /// Opens the output file and writes headers if provided.
    pub fn to_writer(&self) -> Result<StateWriter<State>, Box<dyn Error>> {
        let file = File::create(&self.path)?;
        let mut writer = Writer::from_writer(BufWriter::new(file));

        if !self.headers.is_empty() {
            writer.write_record(&self.headers)?;
            writer.flush()?;
        }

        let writer = StateWriter::new(writer, self.formatter.box_clone());

        Ok(writer)
    }
}

/// A CSV writer for logging state over time using a custom formatter.
pub struct StateWriter<State>
where
    State: Integrable + 'static,
{
    writer: Writer<BufWriter<File>>,
    formatter: Box<dyn StateFormatter<State>>,
    string_buffer: Vec<String>,
    _phantom_data: PhantomData<State>,
}

impl<State> StateWriter<State>
where
    State: Integrable,
{
    /// Constructs a new `StateWriter`.
    ///
    /// # Arguments
    ///
    /// * `writer` - A CSV writer.
    /// * `formatter` - The formatter to convert state data into CSV rows.
    pub fn new(writer: Writer<BufWriter<File>>, formatter: Box<dyn StateFormatter<State>>) -> Self {
        let string_buffer = vec![];
        Self {
            writer,
            formatter,
            string_buffer,
            _phantom_data: PhantomData,
        }
    }

    /// Write a single state and timestamp to the CSV output.
    ///
    /// # Arguments
    ///
    /// * `t` - The current simulation time.
    /// * `state` - The state at time `t`.
    pub fn write(&mut self, t: f64, state: &State) -> Result<(), Box<dyn Error>> {
        // Clear previous entries from the string buffer
        for s in &mut self.string_buffer {
            s.clear();
        }
        // Format the new row
        self.formatter.format(t, state, &mut self.string_buffer)?;
        // Write to CSV
        self.writer.write_record(&self.string_buffer)?;

        Ok(())
    }

    /// Flush the underlying writer to ensure all data is written to disk.
    pub fn flush(&mut self) -> Result<(), Box<dyn Error>> {
        self.writer.flush()?;

        Ok(())
    }
}

impl<State> fmt::Debug for StateWriter<State>
where
    State: Integrable + 'static,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("StateWriter")
            .field("writer", &"Writer<BufWriter<File>>")
            .field("formatter", &"<formatter function>")
            .field("string_buffer", &self.string_buffer)
            .finish()
    }
}

/// Trait for converting state and time values into a string-based format suitable for CSV.
///
/// This trait enables custom formatting of state variables, useful for generic logging.
pub trait StateFormatter<State>
where
    State: Integrable + 'static,
{
    /// Format the given state and time into a vector of strings for CSV writing.
    ///
    /// # Arguments
    ///
    /// * `time` - Current time.
    /// * `state` - Reference to the current state.
    /// * `buffer` - Mutable buffer to fill with CSV string entries.
    fn format(
        &self,
        time: f64,
        state: &State,
        buffer: &mut Vec<String>,
    ) -> Result<(), Box<dyn Error>>;

    /// Clone this formatter as a boxed trait object.
    fn box_clone(&self) -> Box<dyn StateFormatter<State> + 'static>;
}

impl<F, State> StateFormatter<State> for F
where
    State: Integrable + 'static,
    F: Fn(f64, &State, &mut Vec<String>) -> Result<(), Box<dyn Error>> + Clone + 'static,
{
    fn format(
        &self,
        time: f64,
        state: &State,
        buffer: &mut Vec<String>,
    ) -> Result<(), Box<dyn Error>> {
        self(time, state, buffer)
    }

    fn box_clone(&self) -> Box<dyn StateFormatter<State> + 'static> {
        Box::new(self.clone())
    }
}
