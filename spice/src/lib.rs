use bincode;
use pck::{EarthParameters, MoonParameters};
use rotations::{
    euler_angles::{EulerAngles, EulerSequence},
    Rotation,
};
use serde::{Deserialize, Serialize};
use spk::SpiceSpk;

use std::fs::File;
use std::io::{Error as IoError, Write};

pub mod daf;
pub mod pck;
pub mod spk;

pub enum SpiceFileTypes {
    Pck,
    Spk,
}
#[derive(Debug, Deserialize, Serialize)]
pub struct Spice {
    pub earth: EarthParameters,
    moon: MoonParameters,
    spk: SpiceSpk,
}

impl Spice {
    pub fn from_naif() -> Result<Self, SpiceErrors> {
        let earth = EarthParameters::from_naif()?;
        let moon = MoonParameters::from_naif()?;
        let spk = SpiceSpk::from_naif()?;
        Ok(Self { earth, moon, spk })
    }

    pub fn calculate_position(&mut self, t: f64, body: Bodies) -> Result<[f64; 3], SpiceErrors> {
        if let Some(segment) = self.spk.get_segment(&body, t)? {
            // earth and moon are in reference to the earth barycenter, adjust to solar system barycenter to match others
            match segment.target {
                Bodies::Earth | Bodies::Moon => {
                    let body_pos = segment.evaluate(t)?;
                    let barycenter = self.spk.get_segment(&Bodies::EarthBarycenter, t)?.unwrap();
                    let barycenter_pos = barycenter.evaluate(t)?;
                    Ok([
                        body_pos.0 + barycenter_pos.0,
                        body_pos.1 + barycenter_pos.1,
                        body_pos.2 + barycenter_pos.2,
                    ])
                }
                _ => {
                    let result = segment.evaluate(t)?;
                    Ok([result.0, result.1, result.2])
                }
            }
        } else {
            return Err(SpiceErrors::BodyNotFound);
        }
    }

    pub fn calculate_orientation(
        &mut self,
        t: f64,
        body: Bodies,
    ) -> Result<(Rotation, f64, f64, f64), SpiceErrors> {
        let segment = match body {
            Bodies::Earth => {
                if let Some(segment) = self.earth.get_segment(&body, t)? {                    
                    segment
                } else {
                    return Err(SpiceErrors::BodyNotFound);
                }
            }
            Bodies::Moon => {
                if let Some(segment) = self.moon.get_segment(&body, t)? {
                    segment
                } else {
                    return Err(SpiceErrors::BodyNotFound);
                }
            }
            _ => return Err(SpiceErrors::NoOrientationDataForThisBody),
        };

        let (ra, dec, gha) = segment.evaluate(t)?;
        let rotation = Rotation::from(EulerAngles::new(ra, dec, gha, EulerSequence::ZXZ));
        Ok((rotation, ra, dec, gha))
    }

    pub fn save_spice_data(&self) -> std::io::Result<()> {
        let encoded: Vec<u8> = bincode::serialize(self).unwrap(); // Serialize the struct
        let mut file = File::create("resources/spice.dat")?;
        file.write_all(&encoded)?; // Write the serialized data to a file
        Ok(())
    }
}

#[derive(Debug)]
pub enum SpiceErrors {
    BodyNotFound,
    IoError(IoError),
    NoOrientationDataForThisBody,
    RecordMetaNotFound,
    RecordNotFound,
    ReqwestError(reqwest::Error),
}

impl From<IoError> for SpiceErrors {
    fn from(e: IoError) -> Self {
        SpiceErrors::IoError(e)
    }
}

impl From<reqwest::Error> for SpiceErrors {
    fn from(e: reqwest::Error) -> Self {
        SpiceErrors::ReqwestError(e)
    }
}

/// Note that for most of these, we use the Barycenter since it's all that's available in the de440s.bsp
/// The only ones that are not barycenter are the earth, moon, and sun. Change this in the future if needed
#[derive(Debug, PartialEq, Eq, Hash, Clone, Copy, Serialize, Deserialize)]
pub enum Bodies {
    SolarSystemBarycenter,
    Mercury,
    MercuryBarycenter,
    Venus,
    VenusBarycenter,
    Earth,
    EarthBarycenter,
    Moon,
    Mars,
    MarsBarycenter,
    Jupiter,
    JupiterBarycenter,
    Saturn,
    SaturnBarycenter,
    Neptune,
    NeptuneBarycenter,
    Uranus,
    UranusBarycenter,
    Pluto,
    PlutoBarycenter,
    Sun,
}

impl Bodies {
    fn from_spk_id(id: i32) -> Option<Self> {
        match id {
            0 => Some(Bodies::SolarSystemBarycenter),
            1 => Some(Bodies::MercuryBarycenter),
            199 => Some(Bodies::Mercury),
            2 => Some(Bodies::VenusBarycenter),
            299 => Some(Bodies::Venus),
            3 => Some(Bodies::EarthBarycenter),
            399 => Some(Bodies::Earth),
            301 => Some(Bodies::Moon),
            4 => Some(Bodies::MarsBarycenter),
            499 => Some(Bodies::Mars),
            5 => Some(Bodies::JupiterBarycenter),
            599 => Some(Bodies::Jupiter),
            6 => Some(Bodies::SaturnBarycenter),
            699 => Some(Bodies::Saturn),
            7 => Some(Bodies::UranusBarycenter),
            799 => Some(Bodies::Uranus),
            8 => Some(Bodies::NeptuneBarycenter),
            899 => Some(Bodies::Neptune),
            9 => Some(Bodies::PlutoBarycenter),
            999 => Some(Bodies::Pluto),
            10 => Some(Bodies::Sun),
            _ => {
                dbg!(id);
                None
            }
        }
    }

    fn from_pck_id(id: i32) -> Option<Self> {
        match id {
            3000 => Some(Bodies::Earth),
            31008 => Some(Bodies::Moon),
            _ => {
                dbg!(id);
                None
            }
        }
    }

    pub fn from_string(str: &String) -> Option<Self> {
        match str.to_lowercase().as_str() {
            "solar system barycenter" => Some(Bodies::SolarSystemBarycenter),
            "mercury barycenter" => Some(Bodies::MercuryBarycenter),
            "mercury" => Some(Bodies::Mercury),
            "venus barycenter" => Some(Bodies::VenusBarycenter),
            "venus" => Some(Bodies::Venus),
            "earth barycenter" => Some(Bodies::EarthBarycenter),
            "earth" => Some(Bodies::Earth),
            "moon" => Some(Bodies::Moon),
            "mars barycenter" => Some(Bodies::MarsBarycenter),
            "mars" => Some(Bodies::Mars),
            "jupiter barycenter" => Some(Bodies::JupiterBarycenter),
            "jupiter" => Some(Bodies::Jupiter),
            "saturn barycenter" => Some(Bodies::SaturnBarycenter),
            "saturn" => Some(Bodies::Saturn),
            "uranus barycenter" => Some(Bodies::UranusBarycenter),
            "uranus" => Some(Bodies::Uranus),
            "neptune barycenter" => Some(Bodies::NeptuneBarycenter),
            "neptune" => Some(Bodies::Neptune),
            "pluto barycenter" => Some(Bodies::PlutoBarycenter),
            "pluto" => Some(Bodies::Pluto),
            "sun" => Some(Bodies::Sun),
            _ => {
                println!("Error: body '{}' not found", str);
                println!("Note: you may need to use quotes like 'earth barycenter'");
                println!("Options:");
                println!("solar system barycenter");
                println!("mercury barycenter");
                println!("mercury");
                println!("venus barycenter");
                println!("venus");
                println!("earth barycenter");
                println!("earth");
                println!("moon");
                println!("mars barycenter");
                println!("mars");
                println!("jupiter barycenter");
                println!("jupiter");
                println!("saturn barycenter");
                println!("saturn");
                println!("uranus barycenter");
                println!("uranus");
                println!("neptune barycenter");
                println!("neptune");
                println!("pluto barycenter");
                println!("pluto");
                println!("sun");
                None
            }
        }
    }
}

/// Lets just stick with J2000 for now
#[derive(Debug, Deserialize, Serialize)]
enum Frames {
    J2000Equatorial,
    J2000Ecliptic,
}

impl Frames {
    fn from_id(id: i32) -> Option<Self> {
        match id {
            1 => Some(Frames::J2000Equatorial),
            17 => Some(Frames::J2000Ecliptic),
            _ => None,
        }
    }
}

#[derive(Debug, Deserialize, Serialize)]
enum DataTypes {
    ModifiedDifferenceArrays,
    ChebyshevPolyPosition,
    ChebyshevPolyPositionVelocity,
    DiscretePositionVelocity,
    TabularPositionVelocity,
    HighPrecisionChevyshevPoly,
    EquinocatalOrbitalElements,
}

impl DataTypes {
    fn from_id(id: i32) -> Option<Self> {
        match id {
            1 => Some(DataTypes::ModifiedDifferenceArrays),
            2 => Some(DataTypes::ChebyshevPolyPosition),
            3 => Some(DataTypes::ChebyshevPolyPositionVelocity),
            5 => Some(DataTypes::DiscretePositionVelocity),
            9 => Some(DataTypes::TabularPositionVelocity),
            13 => Some(DataTypes::HighPrecisionChevyshevPoly),
            15 => Some(DataTypes::EquinocatalOrbitalElements),
            _ => None,
        }
    }
}
