use std::{
    ops::{AddAssign, Deref, DerefMut, MulAssign},
    path::PathBuf,
};

use tolerance::{Tolerance, Tolerances, compute_error};

use crate::Integrable;

#[derive(Clone, Copy)]
pub struct StateArray<const N: usize>([f64; N]);

impl<const N: usize> StateArray<N> {
    pub fn new(array: [f64; N]) -> Self {
        Self(array)
    }
}

impl<const N: usize> Default for StateArray<N> {
    fn default() -> Self {
        Self([0.0; N])
    }
}

impl<const N: usize> AddAssign<&Self> for StateArray<N> {
    fn add_assign(&mut self, rhs: &Self) {
        for i in 0..N {
            self.0[i] += rhs.0[i];
        }
    }
}

impl<const N: usize> MulAssign<f64> for StateArray<N> {
    fn mul_assign(&mut self, rhs: f64) {
        for i in 0..N {
            self.0[i] *= rhs;
        }
    }
}

impl<const N: usize> Integrable for StateArray<N> {
    type Derivative = Self;
    type Tolerance = StateArrayTolerances<N>;

    fn initialize_writer(path: &PathBuf) -> Option<csv::Writer<std::io::BufWriter<std::fs::File>>> {
        // Create a new path by joining the directory path with the filename
        let file_path = path.join("result.csv");

        // Ensure the directory exists
        if let Some(parent) = file_path.parent() {
            if !parent.exists() {
                if let Err(err) = std::fs::create_dir_all(parent) {
                    eprintln!("Failed to create directory {:?}: {}", parent, err);
                    return None;
                }
            }
        }
        match std::fs::File::create(&file_path) {
            Ok(file) => {
                let mut writer = csv::Writer::from_writer(std::io::BufWriter::new(file));

                // Create header vector
                let mut headers = Vec::with_capacity(N + 1);
                headers.push("t".to_string());

                // Add element headers
                for i in 0..N {
                    headers.push(format!("x[{}]", i));
                }

                // Write headers
                if let Err(err) = writer.write_record(&headers) {
                    eprintln!("Failed to write headers: {}", err);
                    return None;
                }

                // Flush to ensure headers are written
                if let Err(err) = writer.flush() {
                    eprintln!("Failed to flush writer: {}", err);
                    return None;
                }

                Some(writer)
            }
            Err(err) => {
                eprintln!("Failed to create file at {:?}: {}", path, err);
                None
            }
        }
    }

    fn save_to_writer(&self, writer: &mut csv::Writer<std::io::BufWriter<std::fs::File>>, t: f64) {
        // Using the serializer approach avoids string conversions
        let mut record = vec![t];

        // Add each element directly as f64
        for i in 0..N {
            record.push(self.0[i]);
        }

        // Serialize and write the record
        if let Err(err) = writer.serialize(&record) {
            eprintln!("Failed to serialize record at t={}: {}", t, err);
        }
    }
}

impl<const N: usize> Deref for StateArray<N> {
    type Target = [f64; N];

    fn deref(&self) -> &Self::Target {
        &self.0
    }
}

impl<const N: usize> DerefMut for StateArray<N> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.0
    }
}

pub struct StateArrayTolerances<const N: usize>([Option<Tolerances>; N]);
impl<const N: usize> Tolerance for StateArrayTolerances<N> {
    type State = StateArray<N>;
    fn compute_error(
        &self,
        x0: &StateArray<N>,
        xf: &StateArray<N>,
        rel_tol: f64,
        abs_tol: f64,
    ) -> f64 {
        let mut sum_squared_errors = 0.0;
        let mut count = 0;

        // Compute the squared error for each component
        for (i, tol) in self.0.iter().enumerate() {
            let component_error = if let Some(tol) = tol {
                tol.compute_error(x0.0[i], xf.0[i])
            } else {
                compute_error(x0.0[i], xf.0[i], rel_tol, abs_tol)
            };

            sum_squared_errors += component_error * component_error;
            count += 1;
        }

        // If no components were checked, return 0.0
        if count == 0 {
            return 0.0;
        }

        // Return the root mean square error
        (sum_squared_errors / count as f64).sqrt()
    }
}

impl<const N: usize> Default for StateArrayTolerances<N> {
    fn default() -> Self {
        Self([None; N])
    }
}
