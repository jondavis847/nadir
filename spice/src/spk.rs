use crate::{SpiceBodies, SpiceFileTypes};
use std::io::{self, Write};
use super::{
    daf::{DafData, Segment},
    SpiceErrors,
};
use serde::{Deserialize, Serialize};
use std::path::PathBuf;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SpiceSpk(pub DafData);

impl SpiceSpk {
    pub fn get_segment(&mut self, body: &SpiceBodies, t: f64) -> Result<Option<&mut Segment>,SpiceErrors> {
        self.0.get_segment(body, t)
    }

    pub fn from_local(path: &PathBuf) -> Result<Self, SpiceErrors> {
        let start = std::time::Instant::now();
        let path_str = path.to_str().unwrap();
        print!("Loading file from {path_str}...");
        io::stdout().flush()?;

        // Read the entire file into a Vec<u8>
        let spk_bytes = std::fs::read(path)?;
        let duration = start.elapsed();
        let daf_data = DafData::new(&spk_bytes, SpiceFileTypes::Spk)?;
        println!("Done! in {} sec.", duration.as_secs_f32());
        Ok(Self(daf_data))
    }

    pub fn from_naif() -> Result<Self, SpiceErrors> {
        let start = std::time::Instant::now();
        print!("Getting spk file 'de440s.bsp' from naif website...");
        io::stdout().flush()?;
        let spk_url = "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440s.bsp";
        let response = reqwest::blocking::get(spk_url)?;
        let spk_bytes = response.bytes()?;
        let duration = start.elapsed();
        let daf_data = DafData::new(&spk_bytes, SpiceFileTypes::Spk)?;
        println!("Done! in {} sec.", duration.as_secs_f32());
        Ok(Self(daf_data))
    }
}
