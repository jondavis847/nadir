use std::{collections::HashMap, fs::File, io::BufWriter, path::PathBuf};

use ambassador::delegatable_trait;
use csv::Writer;

pub type ResultWriter = Writer<BufWriter<File>>;

pub struct ResultManager {
    writers: HashMap<u32, ResultWriter>,
    pub result_path: PathBuf,
    next_id: u32,
}

impl ResultManager {
    pub fn new(result_path: PathBuf) -> Self {
        Self { writers: HashMap::new(), result_path, next_id: 0 }
    }

    pub fn new_writer(&mut self, name: &str, path: &PathBuf, headers: &[&str]) -> u32 {
        // Ensure the directory exists
        std::fs::create_dir_all(path).expect("Failed to create directory");

        let filename = name.to_string() + ".csv";
        let file = File::create(path.join(filename)).expect("Failed to create file");
        let buf_writer = BufWriter::new(file);
        let mut writer = Writer::from_writer(buf_writer);
        writer
            .write_record(headers)
            .expect("Failed to write headers");
        let id = self.next_id;
        self.writers
            .insert(id, writer);
        self.next_id += 1;
        id
    }

    pub fn write_record(&mut self, id: u32, content: &[String]) {
        let writer = self
            .writers
            .get_mut(&id)
            .expect("Writer not found");
        writer
            .write_record(content)
            .expect("Failed to write record");
    }

    pub fn flush(&mut self) {
        for writer in self
            .writers
            .values_mut()
        {
            writer
                .flush()
                .expect("Failed to flush writer");
        }
    }
}

#[delegatable_trait]
pub trait NadirResult {
    /// Initializes the ResultEntry for storing the sim result for this joint
    fn new_result(&mut self, results: &mut ResultManager);
    // Writes the next entry in the result file
    fn write_result(&self, results: &mut ResultManager);
}
