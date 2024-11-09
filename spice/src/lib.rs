use bincode;
use dirs::config_dir;
use nalgebra::Vector3;
use pck::{EarthParameters, MoonParameters};
use reqwest::blocking::Client;
use rotations::{
    euler_angles::{EulerAngles, EulerSequence},
    quaternion::Quaternion,
    Rotation, RotationTrait,
};
use serde::{Deserialize, Serialize};
use spk::SpiceSpk;

use std::fs::File;
use std::io::{Read, Write};
use time::{Time, TimeSystem};

pub mod daf;
pub mod pck;
pub mod spk;

#[derive(Debug)]
pub enum SpiceErrors {
    BodyNotFound,
    CantOpenConfigDir,    
    HeaderNotFound,    
    NoOrientationDataForThisBody,
    RecordMetaNotFound,
    RecordNotFound,    
}

impl std::fmt::Display for SpiceErrors {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            SpiceErrors::BodyNotFound => writeln!(f,"Body not found in Spice data."),
            SpiceErrors::CantOpenConfigDir => writeln!(f,"Could not open the computer's config directory."),
            SpiceErrors::HeaderNotFound => writeln!(f,"Header not found."),
            SpiceErrors::NoOrientationDataForThisBody => writeln!(f,"Could not find orientation data for this body."),
            SpiceErrors::RecordMetaNotFound => writeln!(f,"Record meta not found."),
            SpiceErrors::RecordNotFound => writeln!(f,"Record not found."),
        }
    }
}
impl std::error::Error for SpiceErrors {}

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
    /// Attempts to load the file from the serialize binary in the local temp directory
    /// If for any reason it can't, it will attempt to redownload and save the latest files
    /// It will also check the dates of the spice files and update if possible
    pub fn from_local() -> Result<Self, Box<dyn std::error::Error>> {
        // can we find the config dir where data is saved?
        if let Some(mut path) = config_dir() {
            path.push("nadir/spice.data");
            // does the file already exist?
            if path.exists() {
                let mut file = File::open(&path)?;
                let mut buffer = Vec::new();
                file.read_to_end(&mut buffer)?;
                // atempt to deserialize, if error redownload new files
                let mut spice: Spice = match bincode::deserialize(&buffer) {
                    Ok(spice) => spice,
                    Err(e) => {
                        println!("{e}");
                        Spice::from_naif()?
                    }
                };
                // check if the files are the most recent, if not get new ones and resave
                if !spice.earth.check_naif()? {
                    let earth = EarthParameters::from_naif();
                    match earth {
                        Ok(earth) => spice.earth = earth,
                        Err(e) => {
                            println!("{:?}", e);
                            println!("using previous earth values");
                        }
                    }
                }
                if !spice.moon.check_naif()? {
                    let moon = MoonParameters::from_naif();
                    match moon {
                        Ok(moon) => spice.moon = moon,
                        Err(e) => {
                            println!("{:?}", e);
                            println!("using previous moon values");
                        }
                    }
                }
                if !spice.spk.check_naif()? {
                    let spk = SpiceSpk::from_naif();
                    match spk {
                        Ok(spk) => spice.spk = spk,
                        Err(e) => {
                            println!("{:?}", e);
                            println!("using previous spk values");
                        }
                    }
                }
                Ok(spice)
            } else {
                println!("No spice data found locally, downloading...");
                Spice::from_naif()
            }
        } else {
            Err(SpiceErrors::CantOpenConfigDir.into())
        }
    }

    pub fn from_naif() -> Result<Self, Box<dyn std::error::Error>> {
        let earth = EarthParameters::from_naif()?;
        let moon = MoonParameters::from_naif()?;
        let spk = SpiceSpk::from_naif()?;
        let spice = Self { earth, moon, spk };
        spice.save_spice_data()?;
        Ok(spice)
    }

    pub fn calculate_position(
        &mut self,
        t: Time,
        body: SpiceBodies,
    ) -> Result<Vector3<f64>, Box<dyn std::error::Error>> {
        let t = t.to_system(TimeSystem::TT).get_seconds_j2k();
        // earth is the center of our frame, so always zeros
        if body == SpiceBodies::Earth {
            return Ok(Vector3::zeros());
        }
        // we use gcrf as our frame, so everything gets referenced to earth's center
        let earth_barycenter_segment = self
            .spk
            .get_segment(&SpiceBodies::EarthBarycenter, t)?
            .unwrap();
        // earth barycenter position referenced to solar system barycenter
        let earth_barycenter_ssb = earth_barycenter_segment.evaluate(t)?;

        let earth_segment = self.spk.get_segment(&SpiceBodies::Earth, t)?.unwrap();
        // earth body position referenced to earth barycenter
        let earth_position_eb = earth_segment.evaluate(t)?;

        if let Some(segment) = self.spk.get_segment(&body, t)? {
            match segment.target {
                SpiceBodies::Moon => {
                    // moon is the only other one that is referenced to earth's barycenter
                    let moon_position_eb = segment.evaluate(t)?;

                    // earth pos barycenter is vector from barycenter to earth, make negative for vector from earth to barycenter
                    let moon_position_j2000 = -earth_position_eb + moon_position_eb;

                    Ok(moon_position_j2000)
                }
                _ => {
                    //ssb is the solar system barycenter, which spice segments output by default
                    // gather everything in ssb, then convert to GCRF by subtracting earth_position_ssb
                    // orientation of all vectors in ICRF, which is same orientation as GCRF/J2000
                    let earth_position_ssb = earth_barycenter_ssb + earth_position_eb;
                    let body_position_ssb = segment.evaluate(t)?;
                    let result = body_position_ssb - earth_position_ssb;

                    Ok(result)
                }
            }
        } else {
            Err(SpiceErrors::BodyNotFound.into())
        }
    }

    pub fn calculate_orientation(
        &mut self,
        t: Time,
        body: SpiceBodies,
    ) -> Result<Rotation, Box<dyn std::error::Error>> {
        let t = t.to_system(TimeSystem::TT).get_seconds_j2k();
        let segment = match body {
            SpiceBodies::Earth => {
                if let Some(segment) = self.earth.get_segment(&body, t)? {
                    segment
                } else {
                    return Err(SpiceErrors::BodyNotFound.into());
                }
            }
            SpiceBodies::Moon => {
                if let Some(segment) = self.moon.get_segment(&body, t)? {
                    segment
                } else {
                    return Err(SpiceErrors::BodyNotFound.into());
                }
            }
            _ => return Err(SpiceErrors::NoOrientationDataForThisBody.into()),
        };

        //TODO: include century calculation for obliquity for completeness?
        let obliquity = 23.43928111111111 * std::f64::consts::PI / 180.0; // https://ssd.jpl.nasa.gov/astro_par.html
        let j2000_equatorial_from_ecliptic = Rotation::from(Quaternion::new(
            (-obliquity / 2.0).sin(),
            0.0,
            0.0,
            (-obliquity / 2.0).cos(),
        ));

        let result = segment.evaluate(t)?;
        let ra = result[0];
        let dec = result[1];
        let gha = result[2];
    
        //TODO: we take inv because of spice giving passive rotation, we use active
        // fix when we go to passive rotations
        let orientation_ecliptic =
            Rotation::from(EulerAngles::new(ra, dec, gha, EulerSequence::ZXZ)).inv();
        let orientation_equatorial = j2000_equatorial_from_ecliptic * orientation_ecliptic;
        Ok(orientation_equatorial)
    }

    pub fn save_spice_data(&self) -> Result<(), Box<dyn std::error::Error>> {
        if let Some(mut path) = config_dir() {
            path.push("nadir");
            path.push("spice.data");
            let encoded: Vec<u8> = bincode::serialize(self).unwrap(); // Serialize the struct
            let mut file = File::create(path)?;
            file.write_all(&encoded)?; // Write the serialized data to a file
            Ok(())
        } else {
            Err(SpiceErrors::CantOpenConfigDir.into())
        }
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

fn check_naif(url: &str, local: String) -> Result<bool, Box<dyn std::error::Error>> {
    let client = Client::new();
    let response = client.head(url).send()?;
    if let Some(last_modified) = response.headers().get("last-modified") {
        let remote_date = last_modified.to_str().unwrap().to_string();
        if remote_date == local {
            Ok(true)
        } else {
            Ok(false)
        }
    } else {
        Err(SpiceErrors::HeaderNotFound.into())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use time::TimeSystem;

    #[test]
    fn test_earth_position_2000() {
        let mut spice = Spice::from_local().unwrap();
        let epoch = Time::from_sec_j2k(0.0, TimeSystem::TT);
        let result = spice.calculate_position(epoch, SpiceBodies::Earth).unwrap();

        // values obtained from SPICE toolkit (MICE, de440s.bsp)
        //cspice_spkpos('EARTH', 0, 'J2000', 'NONE', 'EARTH')
        assert_abs_diff_eq!(result[0], 0.0, epsilon = 1.0);
        assert_abs_diff_eq!(result[1], 0.0, epsilon = 1.0);
        assert_abs_diff_eq!(result[2], 0.0, epsilon = 1.0);
    }

    #[test]
    fn test_earth_position_2024() {
        let mut spice = Spice::from_local().unwrap();
        let epoch = Time::from_ymdhms(2024, 1, 1, 0, 0, 0.0, TimeSystem::TT).unwrap();
        let result = spice.calculate_position(epoch, SpiceBodies::Earth).unwrap();

        // values obtained from SPICE toolkit (MICE, de440s.bsp)
        //cspice_spkpos('EARTH', 757339200, 'J2000', 'NONE', 'EARTH')
        assert_abs_diff_eq!(result[0], 0.0, epsilon = 1.0);
        assert_abs_diff_eq!(result[1], 0.0, epsilon = 1.0);
        assert_abs_diff_eq!(result[2], 0.0, epsilon = 1.0);
    }

    #[test]
    fn test_moon_position_2000() {
        let mut spice = Spice::from_local().unwrap();
        let epoch = Time::from_sec_j2k(0.0, TimeSystem::TT);
        let result = spice.calculate_position(epoch, SpiceBodies::Moon).unwrap();

        // values obtained from SPICE toolkit (MICE, de440s.bsp)
        //cspice_spkpos('MOON', 0.0, 'J2000', 'NONE', 'EARTH')
        assert_abs_diff_eq!(result[0], -291608.384633435, epsilon = 1.0);
        assert_abs_diff_eq!(result[1], -266716.833394233, epsilon = 1.0);
        assert_abs_diff_eq!(result[2], -76102.487099902, epsilon = 1.0);
    }

    #[test]
    fn test_moon_position_2024() {
        let mut spice = Spice::from_local().unwrap();
        let epoch = Time::from_ymdhms(2024, 1, 1, 0, 0, 0.0, TimeSystem::TT).unwrap();
        let result = spice.calculate_position(epoch, SpiceBodies::Moon).unwrap();

        // values obtained from SPICE toolkit (MICE, de440s.bsp)
        //cspice_spkpos('MOON', 757339200, 'J2000', 'NONE', 'EARTH')
        assert_abs_diff_eq!(result[0], -367952.530290496, epsilon = 1.0);
        assert_abs_diff_eq!(result[1], 142774.974775621, epsilon = 1.0);
        assert_abs_diff_eq!(result[2], 89342.2820186614, epsilon = 1.0);
    }

    #[test]
    fn test_sun_position_2000() {
        let mut spice = Spice::from_local().unwrap();
        let epoch = Time::from_sec_j2k(0.0, TimeSystem::TT);
        let result = spice.calculate_position(epoch, SpiceBodies::Sun).unwrap();

        // values obtained from SPICE toolkit (MICE, de440s.bsp)
        //cspice_spkpos('SUN', 0, 'J2000', 'NONE', 'EARTH')

        assert_abs_diff_eq!(result[0], 26499033.6774251, epsilon = 1.0);
        assert_abs_diff_eq!(result[1], -132757417.338339, epsilon = 1.0);
        assert_abs_diff_eq!(result[2], -57556718.4705382, epsilon = 1.0);
    }

    #[test]
    fn test_sun_position_2024() {
        let mut spice = Spice::from_local().unwrap();
        let epoch = Time::from_ymdhms(2024, 1, 1, 0, 0, 0.0, TimeSystem::TT).unwrap();
        let result = spice.calculate_position(epoch, SpiceBodies::Sun).unwrap();

        // values obtained from SPICE toolkit (MICE, de440s.bsp)
        //cspice_spkpos('SUN', 757339200, 'J2000', 'NONE', 'EARTH')
        assert_abs_diff_eq!(result[0], 24810993.2596643, epsilon = 1.0);
        assert_abs_diff_eq!(result[1], -133033452.131135, epsilon = 1.0);
        assert_abs_diff_eq!(result[2], -57668106.2401691, epsilon = 1.0);
    }
}
