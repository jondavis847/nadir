use serde::{Deserialize, Serialize};
use std::io::{self, Write};

use crate::{
    check_naif,
    daf::{DafData, Segment},
    SpiceBodies, SpiceErrors, SpiceFileTypes,
};
// We just create this data manually from the pck file (pck00011.tpc)
// since it's available in ascii format and there aren't that many parameters
// We will update with EOP files automatically
// These values have an error of up to 150 arcseconds based on the docs, use EOPs or expect errors

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct EarthParameters {
    daf: DafData,
    last_modified: String,
}

impl EarthParameters {
    const EARTH_BPC: &'static str = "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_1962_240827_2124_combined.bpc";

    pub fn check_naif(&self) -> Result<bool, SpiceErrors> {
        check_naif(EarthParameters::EARTH_BPC, self.last_modified.clone())
    }

    pub fn from_naif() -> Result<Self, SpiceErrors> {
        let start = std::time::Instant::now();
        print!("Getting latest eop file from naif website...");
        io::stdout().flush().expect("spice could not flush io");
        let response =
            reqwest::blocking::get(EarthParameters::EARTH_BPC).expect("spice could not http get");

        // Get the Last-Modified header if it exists
        let last_modified = response
            .headers()
            .get("Last-Modified")
            .and_then(|val| val.to_str().ok())
            .map(String::from)
            .ok_or(SpiceErrors::HeaderNotFound)?;

        let eop_bytes = response.bytes().expect("spiace could not get bytes");
        let daf_data = DafData::new(&eop_bytes, SpiceFileTypes::Pck)?;
        let duration = start.elapsed();
        println!("Done! in {} sec.", duration.as_secs_f32());
        Ok(Self {
            daf: daf_data,
            last_modified,
        })
    }

    pub fn get_segment(
        &mut self,
        body: &SpiceBodies,
        t: f64,
    ) -> Result<Option<&mut Segment>, SpiceErrors> {
        self.daf.get_segment(body, t)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MoonParameters {
    daf: DafData,
    last_modified: String,
}

impl MoonParameters {
    const MOON_BPC: &'static str =
        "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/moon_pa_de440_200625.bpc";

    pub fn check_naif(&self) -> Result<bool, SpiceErrors> {
        check_naif(MoonParameters::MOON_BPC, self.last_modified.clone())
    }

    pub fn from_naif() -> Result<Self, SpiceErrors> {
        let start = std::time::Instant::now();
        print!("Getting latest moon file from naif website...");
        io::stdout().flush().expect("spice could not flush io");
        let response = reqwest::blocking::get(MoonParameters::MOON_BPC)
            .expect("spice could not perform http get request");

        // Get the Last-Modified header if it exists
        let last_modified = response
            .headers()
            .get("Last-Modified")
            .and_then(|val| val.to_str().ok())
            .map(String::from)
            .ok_or(SpiceErrors::HeaderNotFound)?;

        let eop_bytes = response.bytes().expect("spice could not get eop bytes");
        let daf_data = DafData::new(&eop_bytes, SpiceFileTypes::Pck)?;
        let duration = start.elapsed();
        println!("Done! in {} sec.", duration.as_secs_f32());
        Ok(Self {
            daf: daf_data,
            last_modified,
        })
    }

    pub fn get_segment(
        &mut self,
        body: &SpiceBodies,
        t: f64,
    ) -> Result<Option<&mut Segment>, SpiceErrors> {
        self.daf.get_segment(body, t)
    }
}
