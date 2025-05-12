use std::{fs::File, io::BufWriter, path::PathBuf};

use csv::Writer;

use crate::Integrable;

#[derive(Debug)]
pub enum ResultStorage<State>
where
    State: Integrable,
{
    Memory(MemoryResult<State>),
    File(Writer<BufWriter<File>>),
    None,
}

impl<State: Integrable> ResultStorage<State> {
    pub fn save(&mut self, t: f64, y: &State) {
        match self {
            ResultStorage::Memory(result) => {
                result.insert(t, y);
            }
            ResultStorage::File(writer) => y.save_to_writer(writer, t),
            _ => {}
        }
    }

    pub fn truncate(&mut self) {
        match self {
            ResultStorage::Memory(result) => {
                result.truncate();
            }
            _ => {}
        }
    }
}

#[derive(Debug)]
pub struct MemoryResult<State>
where
    State: Integrable,
{
    pub t: Vec<f64>,
    pub y: Vec<State>,
    i: usize, // current index
}

impl<State: Integrable> MemoryResult<State> {
    pub fn new(n: usize) -> Self {
        Self {
            t: vec![0.0; n],
            y: vec![State::default(); n],
            i: 0,
        }
    }
    fn insert(&mut self, t: f64, x: &State) {
        if self.i == self.t.len() - 1 {
            self.extend();
        }
        self.t[self.i] = t;
        self.y[self.i].clone_from(x);
        self.i += 1;
    }

    fn len(&self) -> usize {
        self.t.len()
    }

    // doubles the length if capapcity is reached
    fn extend(&mut self) {
        self.t.extend(vec![0.0; self.len()]);
        self.y.extend(vec![State::default(); self.len()]);
    }

    fn truncate(&mut self) {
        self.t.truncate(self.i);
        self.y.truncate(self.i);
    }
}
