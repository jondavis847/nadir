use crate::{
    geomag::GeoMagnetism,
    gravity::{Gravity, GravityTrait, NewtownianGravity},
};
use nalgebra::Vector3;
use rotations::{
    prelude::{EulerAngles, EulerSequence, Quaternion},
    RotationTrait,
};
use serde::{Deserialize, Serialize};
use spice::{Spice, SpiceBodies};

use std::{collections::HashMap, f64::consts::PI, mem::take};
use thiserror::Error;
use time::Time;

#[derive(Debug, Error)]
pub enum CelestialErrors {
    #[error("celestial body not found in celestial system")]
    BodyNotFoundInCelestialSystem,
    #[error("celestial body already exists in the celestial system.")]
    CelestialBodyAlreadyExists,
    #[error("spice data not found in celestial system")]
    SpiceNotFound,
    #[error(
        "state found in celestial result - should only be position[x/y/z] or orientation[x/y/z/w]"
    )]
    StateNotFound(String),
}
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CelestialSystem {
    pub epoch: Time,
    pub bodies: Vec<CelestialBody>,
    #[serde(skip)]
    result: CelestialResult,
}

impl CelestialSystem {
    pub fn new(epoch: Time) -> Result<Self, CelestialErrors> {
        // make sure there's at least a sun
        let sun = CelestialBody::new(CelestialBodies::Sun, None, None);
        let bodies = vec![sun];

        Ok(Self {
            epoch,
            bodies,
            result: CelestialResult::default(),
        })
    }

    pub fn update(&mut self, t: f64, spice: &mut Spice) -> Result<(), Box<dyn std::error::Error>> {
        // t is sim time in seconds
        let current_epoch = self.epoch + t;
        let current_epoch = current_epoch.to_system(time::TimeSystem::TT);
        for body in self.bodies.iter_mut() {
            body.update(current_epoch, spice)?;
        }
        Ok(())
    }

    pub fn update_result(&mut self) {
        // update the epoch based on sim time
        self.result
            .epoch_sec_j2k_tai
            .push(self.epoch.get_seconds_j2k());
        // it should be safe to do this enumerated since we initialized the result struct from the sys struct
        for body in &mut self.bodies {
            body.update_result();
        }
    }

    pub fn set_geomag(
        &mut self,
        body: CelestialBodies,
        b: Option<GeoMagnetism>,
    ) -> Result<(), CelestialErrors> {
        if let Some(celestial_body) = self.bodies.iter_mut().find(|cb| cb.body == body) {
            celestial_body.geomag = b;
            Ok(())
        } else {
            Err(CelestialErrors::BodyNotFoundInCelestialSystem)
        }
    }

    pub fn set_gravity(
        &mut self,
        body: CelestialBodies,
        g: Option<Gravity>,
    ) -> Result<(), CelestialErrors> {
        if let Some(celestial_body) = self.bodies.iter_mut().find(|cb| cb.body == body) {
            celestial_body.gravity = g;
            Ok(())
        } else {
            Err(CelestialErrors::BodyNotFoundInCelestialSystem)
        }
    }

    pub fn add_body(
        &mut self,
        body: CelestialBodies,
        gravity: bool,
        _geomag: bool,
    ) -> Result<(), CelestialErrors> {
        // Check if the body already exists in the vector
        if self.bodies.iter().any(|b| b.body == body) {
            return Err(CelestialErrors::CelestialBodyAlreadyExists);
        }

        let gravity_option = if gravity {
            Some(Gravity::Newtownian(NewtownianGravity::from_body(body)))
            // match body {
            //     CelestialBodies::Earth => Some(Gravity::EGM96(EGM96Gravity {})),
            //     _ => Some(Gravity::Newtownian(NewtownianGravity::from_body(body))),
            // }
        } else {
            None
        };

        let geomag_option = None; // for now

        // Create and add the celestial body
        self.bodies
            .push(CelestialBody::new(body, gravity_option, geomag_option));

        Ok(())
    }

    pub fn delete_body(&mut self, body: CelestialBodies) {
        self.bodies.retain(|b| b.body != body);
    }

    /// calculates gravity based on all bodies in the celestial system with a gravity model
    /// position is in the gcrf/j2000 frame
    pub fn calculate_gravity(&self, position: Vector3<f64>) -> Vector3<f64> {
        let mut g_final = Vector3::zeros();

        for body in &self.bodies {
            match body.body {
                CelestialBodies::Earth => {
                    // Special case for Earth
                    if let Some(gravity) = &body.gravity {
                        match gravity {
                            Gravity::EGM96(gravity) => {
                                let position_itrf = body.orientation.rotate(position);
                                let g_itrf = gravity.calculate(position_itrf);
                                let g_gcrf = body.orientation.transform(g_itrf);
                                g_final += g_gcrf;
                            }
                            Gravity::Newtownian(gravity) => g_final += gravity.calculate(position),
                            Gravity::Constant(gravity) => g_final += gravity.calculate(position),
                        }
                    }
                }
                _ => {
                    // General case for other celestial bodies
                    if let Some(gravity) = &body.gravity {
                        g_final += gravity.calculate(position);
                    }
                }
            }
        }
        g_final
    }

    pub fn initialize_result(&mut self, capacity: usize) {
        for body in &mut self.bodies {
            body.initialize_result(capacity);
        }
        self.result.initialize_result(capacity);
    }

    pub fn get_result_entry(&mut self) -> HashMap<String, Vec<f64>> {
        let mut result = HashMap::new();
        result.insert(
            "epoch".to_string(),
            take(&mut self.result.epoch_sec_j2k_tai),
        );
        result
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct CelestialBody {
    pub body: CelestialBodies,
    pub position: Vector3<f64>,  // icrf
    pub orientation: Quaternion, // icrf to itrf
    pub gravity: Option<Gravity>,
    pub geomag: Option<GeoMagnetism>,
    #[serde(skip)]
    result: CelestialBodyResult,
}

impl CelestialBody {
    pub fn new(
        body: CelestialBodies,
        gravity: Option<Gravity>,
        geomag: Option<GeoMagnetism>,
    ) -> Self {
        Self {
            body,
            position: Vector3::zeros(),
            orientation: Quaternion::IDENTITY,
            gravity,
            geomag,
            result: CelestialBodyResult::default(),
        }
    }

    pub fn update(&mut self, t: Time, spice: &mut Spice) -> Result<(), Box<dyn std::error::Error>> {
        //spice uses TDB/TT
        //gsfc planet fact sheet uses UTC
        //TODO: do these time calcs upstream and pass as args so we don't calculate for each body
        let utc = t.to_system(time::TimeSystem::UTC);
        let jdc = utc.get_jd_centuries();
        let sec_j2k = utc.get_seconds_j2k();

        self.position = spice.calculate_position(t, self.body.to_spice())?;

        self.orientation = match self.body {
            CelestialBodies::Earth | CelestialBodies::Moon => {
                spice.calculate_orientation(t, self.body.to_spice())?
            }
            CelestialBodies::Jupiter => {
                from_planet_fact_sheet(268.057, -0.006, 64.495, 0.002, 9.9250, jdc, sec_j2k)
            }
            CelestialBodies::Mars => {
                from_planet_fact_sheet(317.681, -0.106, 52.887, -0.061, 24.6229, jdc, sec_j2k)
            }
            CelestialBodies::Mercury => {
                from_planet_fact_sheet(281.01, -0.033, 61.414, -0.005, 1407.6, jdc, sec_j2k)
            }
            CelestialBodies::Neptune => from_planet_fact_sheet_neptune(jdc, sec_j2k),
            CelestialBodies::Saturn => {
                from_planet_fact_sheet(40.589, -0.036, 83.537, -0.004, 10.656, jdc, sec_j2k)
            }
            CelestialBodies::Uranus => {
                from_planet_fact_sheet(257.311, 0.0, -15.175, 0.0, -17.24, jdc, sec_j2k)
            }
            CelestialBodies::Venus => {
                from_planet_fact_sheet(272.76, 0.0, 61.414, 0.0, -5832.6, jdc, sec_j2k)
            }
            _ => Quaternion::IDENTITY, //TODO: how ot handle other bodies, warn and continue?
        };

        Ok(())
    }

    pub fn get_result_entry(&mut self) -> HashMap<String, Vec<f64>> {
        let mut result = HashMap::new();
        result.insert("attitude[x]".to_string(), take(&mut self.result.qx));
        result.insert("attitude[y]".to_string(), take(&mut self.result.qy));
        result.insert("attitude[z]".to_string(), take(&mut self.result.qz));
        result.insert("attitude[w]".to_string(), take(&mut self.result.qw));
        result.insert("position[x]".to_string(), take(&mut self.result.rx));
        result.insert("position[y]".to_string(), take(&mut self.result.ry));
        result.insert("position[z]".to_string(), take(&mut self.result.rz));
        result
    }

    fn initialize_result(&mut self, capacity: usize) {
        self.result.qx = Vec::with_capacity(capacity);
        self.result.qy = Vec::with_capacity(capacity);
        self.result.qz = Vec::with_capacity(capacity);
        self.result.qw = Vec::with_capacity(capacity);
        self.result.rx = Vec::with_capacity(capacity);
        self.result.ry = Vec::with_capacity(capacity);
        self.result.rz = Vec::with_capacity(capacity);
    }

    fn update_result(&mut self) {
        self.result.qx.push(self.orientation.x);
        self.result.qy.push(self.orientation.y);
        self.result.qz.push(self.orientation.z);
        self.result.qw.push(self.orientation.s);
        self.result.rx.push(self.position[0]);
        self.result.ry.push(self.position[1]);
        self.result.rz.push(self.position[2]);
    }
}

fn from_planet_fact_sheet(
    ra0: f64,
    ra1: f64,
    dec0: f64,
    dec1: f64,
    hrs_in_day: f64,
    julian_centuries: f64,
    sec_j2k: f64,
) -> Quaternion {
    // j2000 orientation
    let ra = ra0 + ra1 * julian_centuries * PI / 180.0;
    let dec = dec0 + dec1 * julian_centuries * PI / 180.0;
    let initial_orientation = Quaternion::from(&EulerAngles::new(ra, dec, 0.0, EulerSequence::ZYX));
    // current orientation based on epoch
    let day = hrs_in_day * 3600.0;
    let rotation_rate = 2.0 * PI / day;
    let rotation = rotation_rate * sec_j2k;
    let orientation_from_rotation =
        Quaternion::from(&EulerAngles::new(rotation, 0.0, 0.0, EulerSequence::ZYX));
    orientation_from_rotation * initial_orientation
}

fn from_planet_fact_sheet_neptune(julian_centuries: f64, sec_j2k: f64) -> Quaternion {
    // j2000 orientation
    let n = (357.85 + 52.316 * julian_centuries) * PI / 180.0;
    let ra = (299.36 + 0.70 * n.sin()) * PI / 180.0;
    let dec = (43.46 - 0.51 * n.cos()) * PI / 180.0;
    let initial_orientation = Quaternion::from(&EulerAngles::new(ra, dec, 0.0, EulerSequence::ZYX));
    // current orientation based on epoch
    let day = 16.11 * 3600.0;
    let rotation_rate = 2.0 * PI / day;
    let rotation = rotation_rate * sec_j2k;
    let orientation_from_rotation =
        Quaternion::from(&EulerAngles::new(rotation, 0.0, 0.0, EulerSequence::ZYX));
    orientation_from_rotation * initial_orientation
}

#[derive(Debug, Clone, Default, Deserialize, Serialize)]
pub struct CelestialBodyResult {
    rx: Vec<f64>,
    ry: Vec<f64>,
    rz: Vec<f64>,
    qx: Vec<f64>,
    qy: Vec<f64>,
    qz: Vec<f64>,
    qw: Vec<f64>,
}

#[derive(Debug, Clone, Copy, Deserialize, Serialize, PartialEq, Eq, Hash)]
pub enum CelestialBodies {
    Earth,
    Jupiter,
    Mercury,
    Mars,
    Moon,
    Neptune,
    Pluto,
    Saturn,
    Sun,
    Uranus,
    Venus,
}

impl CelestialBodies {
    pub fn to_spice(&self) -> SpiceBodies {
        match self {
            CelestialBodies::Earth => SpiceBodies::Earth,
            CelestialBodies::Jupiter => SpiceBodies::Jupiter,
            CelestialBodies::Mars => SpiceBodies::Mars,
            CelestialBodies::Mercury => SpiceBodies::Mercury,
            CelestialBodies::Moon => SpiceBodies::Moon,
            CelestialBodies::Neptune => SpiceBodies::Neptune,
            CelestialBodies::Pluto => SpiceBodies::Pluto,
            CelestialBodies::Sun => SpiceBodies::Sun,
            CelestialBodies::Saturn => SpiceBodies::Saturn,
            CelestialBodies::Uranus => SpiceBodies::Uranus,
            CelestialBodies::Venus => SpiceBodies::Venus,
        }
    }

    pub fn get_name(&self) -> String {
        match self {
            CelestialBodies::Earth => "earth".to_string(),
            CelestialBodies::Jupiter => "jupiter".to_string(),
            CelestialBodies::Mars => "mars".to_string(),
            CelestialBodies::Mercury => "mercury".to_string(),
            CelestialBodies::Moon => "moon".to_string(),
            CelestialBodies::Neptune => "neptune".to_string(),
            CelestialBodies::Pluto => "pluto".to_string(),
            CelestialBodies::Saturn => "saturn".to_string(),
            CelestialBodies::Sun => "sun".to_string(),
            CelestialBodies::Uranus => "uranus".to_string(),
            CelestialBodies::Venus => "venus".to_string(),
        }
    }

    /// Return the gravitional parameter for the body in m^3/sec^2
    pub fn get_mu(&self) -> f64 {
        match self {
            CelestialBodies::Earth => 3.986004418e14,
            CelestialBodies::Jupiter => 1.26686534e17,
            CelestialBodies::Mars => 4.282837e13,
            CelestialBodies::Mercury => 2.2032e13,
            CelestialBodies::Moon => 4.9048695e12,
            CelestialBodies::Neptune => 6.836529e15,
            CelestialBodies::Pluto => 8.71e11,
            CelestialBodies::Saturn => 3.7931187e16,
            CelestialBodies::Sun => 1.32712440018e20,
            CelestialBodies::Uranus => 5.793939e15,
            CelestialBodies::Venus => 3.24859e14,
        }
    }

    /// Returns the volumetric radius (average of polar and equatorial) of the body in m
    pub fn get_radius(&self) -> f64 {
        match self {
            CelestialBodies::Earth => 6378137.0,
            CelestialBodies::Jupiter => 69911000.0,
            CelestialBodies::Mars => 3389500.0,
            CelestialBodies::Mercury => 2439700.0,
            CelestialBodies::Moon => 1737400.0,
            CelestialBodies::Neptune => 24622000.0,
            CelestialBodies::Pluto => 1188000.0,
            CelestialBodies::Saturn => 58232000.0,
            CelestialBodies::Sun => 695700000.0,
            CelestialBodies::Uranus => 25362000.0,
            CelestialBodies::Venus => 6051800.0,
        }
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CelestialResult {
    pub epoch_sec_j2k_tai: Vec<f64>,
}

impl CelestialResult {
    fn initialize_result(&mut self, capacity: usize) {
        self.epoch_sec_j2k_tai = Vec::with_capacity(capacity);
    }
}
