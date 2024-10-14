use serde::{Deserialize, Serialize};
use std::io::{self,Write};

use crate::{
    daf::{DafData, Segment},
    SpiceBodies, SpiceErrors, SpiceFileTypes,
};
// We just create this data manually from the pck file (pck00011.tpc)
// since it's available in ascii format and there aren't that many parameters
// We will update with EOP files automatically
// These values have an error of up to 150 arcseconds based on the docs, use EOPs or expect errors

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct EarthParameters(pub DafData);

impl EarthParameters {
    pub fn from_naif() -> Result<Self, SpiceErrors> {
        let start = std::time::Instant::now();
        print!("Getting latest eop file from naif website...");
        io::stdout().flush()?;
        let eop_url =
        "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_1962_240827_2124_combined.bpc";
        //"https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/earth_latest_high_prec.bpc";
        let response = reqwest::blocking::get(eop_url)?;
        let eop_bytes = response.bytes()?;
        let duration = start.elapsed();
        let daf_data = DafData::new(&eop_bytes, SpiceFileTypes::Pck)?;
        println!("Done! in {} sec.", duration.as_secs_f32());
        Ok(Self(daf_data))
    }

    pub fn get_segment(
        &mut self,
        body: &SpiceBodies,
        t: f64,
    ) -> Result<Option<&mut Segment>, SpiceErrors> {
        self.0.get_segment(body, t)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct MoonParameters(DafData);

impl MoonParameters {
    pub fn from_naif() -> Result<Self, SpiceErrors> {
        let start = std::time::Instant::now();
        print!("Getting latest moon file from naif website...");
        io::stdout().flush()?;
        let moon_url =
            "https://naif.jpl.nasa.gov/pub/naif/generic_kernels/pck/moon_pa_de440_200625.bpc";
        let response = reqwest::blocking::get(moon_url)?;
        let moon_bytes = response.bytes()?;
        let duration = start.elapsed();
        let daf_data = DafData::new(&moon_bytes, SpiceFileTypes::Pck)?;
        println!("Done! in {} sec.", duration.as_secs_f32());
        Ok(Self(daf_data))
    }

    pub fn get_segment(
        &mut self,
        body: &SpiceBodies,
        t: f64,
    ) -> Result<Option<&mut Segment>, SpiceErrors> {
        self.0.get_segment(body, t)
    }
}
