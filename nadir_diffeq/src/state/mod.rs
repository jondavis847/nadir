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
pub mod state_vectors;

pub trait Integrable: Sized + Clone + Default + MulAssign<f64> + Debug
where
    for<'a> Self: AddAssign<&'a Self> + AddAssign<&'a Self::Derivative>,
{
    type Derivative: Clone + MulAssign<f64> + Sized + Default + Debug;
    type Tolerance: Tolerance<State = Self>;

    fn writer(path: PathBuf) -> StateWriterBuilder<Self>;
}

pub struct StateWriterBuilder<State>
where
    State: Integrable,
{
    ncols: usize,
    path: PathBuf,
    headers: Vec<String>,
    formatter: Box<dyn StateFormatter<State>>,
    _phantom_data: PhantomData<State>,
}

impl<State> StateWriterBuilder<State>
where
    State: Integrable + 'static,
{
    pub fn new<F>(path: PathBuf, ncols: usize, formatter: F) -> Self
    where
        F: StateFormatter<State> + 'static,
    {
        Self {
            ncols,
            path,
            headers: Vec::new(),
            formatter: Box::new(formatter),
            _phantom_data: PhantomData,
        }
    }

    pub fn with_headers<const N: usize>(
        mut self,
        headers: [&str; N],
    ) -> Result<Self, Box<dyn Error>> {
        if self.ncols != N {
            return Err(
                format!("size of headers({N}) does not match ncols({})", self.ncols).into(),
            );
        }
        let headers: Vec<String> = headers.iter().map(|h| h.to_string()).collect();
        self.headers = headers;
        Ok(self)
    }

    pub fn to_writer(&self) -> Result<StateWriter<State>, Box<dyn Error>> {
        let file = File::create(&self.path)?;
        let mut writer = Writer::from_writer(BufWriter::new(file));

        if !self.headers.is_empty() {
            writer.write_record(&self.headers)?;
            writer.flush()?;
        }

        let writer = StateWriter::new(writer, self.ncols, self.formatter.box_clone());

        Ok(writer)
    }
}

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
    pub fn new(
        writer: Writer<BufWriter<File>>,
        ncols: usize,
        formatter: Box<dyn StateFormatter<State>>,
    ) -> Self {
        let string_buffer = vec![String::new(); ncols];
        Self {
            writer,
            formatter,
            string_buffer,
            _phantom_data: PhantomData,
        }
    }
    // Write using a formatter function that reuses the string buffer
    pub fn write(&mut self, t: f64, state: &State) -> Result<(), Box<dyn Error>> {
        // Clear each string in the buffer (without deallocating)
        for s in &mut self.string_buffer {
            s.clear();
        }
        // Let the formatter populate our buffer
        self.formatter.format(t, state, &mut self.string_buffer)?;
        // Write to CSV
        self.writer.write_record(&self.string_buffer)?;

        Ok(())
    }

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

/// Trait for formatting states into string representations
pub trait StateFormatter<State>
where
    State: Integrable + 'static,
{
    /// Format a state at a given time into the provided string buffer
    fn format(
        &self,
        time: f64,
        state: &State,
        buffer: &mut Vec<String>,
    ) -> Result<(), Box<dyn Error>>;

    /// Provide a way to clone boxed trait objects
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
        // Call the closure
        self(time, state, buffer)
    }

    fn box_clone(&self) -> Box<dyn StateFormatter<State> + 'static> {
        Box::new(self.clone())
    }
}
