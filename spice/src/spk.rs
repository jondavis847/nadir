use super::{
    daf::{DafData, Segment},
    SpiceErrors,
};
use crate::{SpiceBodies, SpiceFileTypes, check_naif};
use serde::{Deserialize, Serialize};
use std::io::{self, Write};

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct SpiceSpk{
    daf: DafData,
    last_modified: String,
}

impl SpiceSpk {

    const SPK_BPC: &'static str = "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/spk/planets/de440s.bsp";

    pub fn check_naif(&self) -> Result<bool, SpiceErrors> {
        check_naif(SpiceSpk::SPK_BPC, self.last_modified.clone())
    }

    pub fn get_segment(
        &mut self,
        body: &SpiceBodies,
        t: f64,
    ) -> Result<Option<&mut Segment>, SpiceErrors> {
        self.daf.get_segment(body, t)
    }

    pub fn from_naif() -> Result<Self, SpiceErrors> {
        let start = std::time::Instant::now();
        print!("Getting latest eop file from naif website...");
        io::stdout().flush()?;
        let response = reqwest::blocking::get(SpiceSpk::SPK_BPC)?;

        // Get the Last-Modified header if it exists
        let last_modified = response
            .headers()
            .get("Last-Modified")
            .and_then(|val| val.to_str().ok())
            .map(String::from)
            .ok_or(SpiceErrors::HeaderNotFound)?;

        let eop_bytes = response.bytes()?;
        let daf_data = DafData::new(&eop_bytes, SpiceFileTypes::Spk)?;
        let duration = start.elapsed();
        println!("Done! in {} sec.", duration.as_secs_f32());
        Ok(Self {
            daf: daf_data,
            last_modified,
        })
    }
}
