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
use time::{Time,TimeSystem};

pub mod daf;
pub mod pck;
pub mod spk;

pub enum SpiceFileTypes {
    Pck,
    Spk,
}
#[derive(Clone, Debug, Deserialize, Serialize)]
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
        
        Ok(Self { earth, moon, spk})
    }

    pub fn calculate_position(&mut self, t: Time, body: SpiceBodies) -> Result<[f64; 3], SpiceErrors> {
        let t = t.to_system(TimeSystem::TT).get_seconds_j2k();
        if let Some(segment) = self.spk.get_segment(&body, t)? {
            // earth and moon are in reference to the earth barycenter, adjust to solar system barycenter to match others
            match segment.target {
                SpiceBodies::Earth | SpiceBodies::Moon => {
                    let body_pos = segment.evaluate(t)?;
                    let barycenter = self.spk.get_segment(&SpiceBodies::EarthBarycenter, t)?.unwrap();
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
        t: Time,
        body: SpiceBodies,
    ) -> Result<(Rotation, f64, f64, f64), SpiceErrors> {
        let t = t.to_system(TimeSystem::TT).get_seconds_j2k();
        let segment = match body {
            SpiceBodies::Earth => {
                if let Some(segment) = self.earth.get_segment(&body, t)? {                    
                    segment
                } else {
                    return Err(SpiceErrors::BodyNotFound);
                }
            }
            SpiceBodies::Moon => {
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
pub enum SpiceBodies {
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

impl SpiceBodies {
    fn from_spk_id(id: i32) -> Option<Self> {
        match id {
            0 => Some(SpiceBodies::SolarSystemBarycenter),
            1 => Some(SpiceBodies::MercuryBarycenter),
            199 => Some(SpiceBodies::Mercury),
            2 => Some(SpiceBodies::VenusBarycenter),
            299 => Some(SpiceBodies::Venus),
            3 => Some(SpiceBodies::EarthBarycenter),
            399 => Some(SpiceBodies::Earth),
            301 => Some(SpiceBodies::Moon),
            4 => Some(SpiceBodies::MarsBarycenter),
            499 => Some(SpiceBodies::Mars),
            5 => Some(SpiceBodies::JupiterBarycenter),
            599 => Some(SpiceBodies::Jupiter),
            6 => Some(SpiceBodies::SaturnBarycenter),
            699 => Some(SpiceBodies::Saturn),
            7 => Some(SpiceBodies::UranusBarycenter),
            799 => Some(SpiceBodies::Uranus),
            8 => Some(SpiceBodies::NeptuneBarycenter),
            899 => Some(SpiceBodies::Neptune),
            9 => Some(SpiceBodies::PlutoBarycenter),
            999 => Some(SpiceBodies::Pluto),
            10 => Some(SpiceBodies::Sun),
            _ => {
                dbg!(id);
                None
            }
        }
    }

    fn from_pck_id(id: i32) -> Option<Self> {
        match id {
            3000 => Some(SpiceBodies::Earth),
            31008 => Some(SpiceBodies::Moon),
            _ => {
                dbg!(id);
                None
            }
        }
    }

    pub fn from_string(str: &String) -> Option<Self> {
        match str.to_lowercase().as_str() {
            "solar system barycenter" => Some(SpiceBodies::SolarSystemBarycenter),
            "mercury barycenter" => Some(SpiceBodies::MercuryBarycenter),
            "mercury" => Some(SpiceBodies::Mercury),
            "venus barycenter" => Some(SpiceBodies::VenusBarycenter),
            "venus" => Some(SpiceBodies::Venus),
            "earth barycenter" => Some(SpiceBodies::EarthBarycenter),
            "earth" => Some(SpiceBodies::Earth),
            "moon" => Some(SpiceBodies::Moon),
            "mars barycenter" => Some(SpiceBodies::MarsBarycenter),
            "mars" => Some(SpiceBodies::Mars),
            "jupiter barycenter" => Some(SpiceBodies::JupiterBarycenter),
            "jupiter" => Some(SpiceBodies::Jupiter),
            "saturn barycenter" => Some(SpiceBodies::SaturnBarycenter),
            "saturn" => Some(SpiceBodies::Saturn),
            "uranus barycenter" => Some(SpiceBodies::UranusBarycenter),
            "uranus" => Some(SpiceBodies::Uranus),
            "neptune barycenter" => Some(SpiceBodies::NeptuneBarycenter),
            "neptune" => Some(SpiceBodies::Neptune),
            "pluto barycenter" => Some(SpiceBodies::PlutoBarycenter),
            "pluto" => Some(SpiceBodies::Pluto),
            "sun" => Some(SpiceBodies::Sun),
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
#[derive(Clone, Debug, Deserialize, Serialize)]
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

#[derive(Clone, Debug, Deserialize, Serialize)]
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
