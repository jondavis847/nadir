use crate::{
    geomag::GeoMagnetism,
    gravity::{Gravity, GravityTrait, NewtownianGravity},
};

use nadir_result::ResultManager;
use nalgebra::Vector3;
use rotations::{
    prelude::{EulerAngles, EulerSequence, Quaternion},
    RotationTrait,
};
use serde::{Deserialize, Serialize};
use spice::{Spice, SpiceBodies, SpiceErrors};

use std::f64::consts::PI;
use thiserror::Error;
use time::Time;

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CelestialEpoch {
    time: Time,
    result_id: Option<u32>,
}

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
    #[error("SpiceError: {0}")]
    SpiceError(#[from] SpiceErrors),
}
#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CelestialSystem {
    pub epoch: CelestialEpoch,
    pub bodies: Vec<CelestialBody>,
    pub spice: Option<Spice>,
}

impl CelestialSystem {
    pub fn new(epoch: Time) -> Result<Self, CelestialErrors> {
        // make sure there's at least a sun for animation
        let sun = CelestialBody::new(CelestialBodies::Sun, None, None);
        let bodies = vec![sun];
        // for now lets default to having spice, in the future add analytical ephem
        let spice = Some(Spice::from_local()?);
        let epoch = CelestialEpoch {
            time: epoch,
            result_id: None,
        };
        Ok(Self {
            epoch,
            bodies,
            spice,
        })
    }

    pub fn update(&mut self, t: f64) -> Result<(), CelestialErrors> {
        if let Some(spice) = &mut self.spice {
            // t is sim time in seconds
            let current_epoch = self.epoch.time + t;
            let current_epoch = current_epoch.to_system(time::TimeSystem::TT);
            for body in self.bodies.iter_mut() {
                body.update(current_epoch, spice)?;
            }
            Ok(())
        } else {
            Err(CelestialErrors::SpiceNotFound)
        }
    }

    /// returns a tuple of bufwriters, the first element is the epoch writer, the second element is a Vec of the writers for the bodies
    pub fn initialize_writers(
        &mut self,
        result: &mut ResultManager,
    ) {
        // Define the celestial subfolder folder path
        let celestial_folder_path = result.result_path.join("celestial");

        // Check if the folder exists, if not, create it
        if !celestial_folder_path.exists() {
            std::fs::create_dir_all(&celestial_folder_path)
                .expect("Failed to create celestial folder");
        }

        for body in &mut self.bodies {
            let headers = [
                "position(base)[x]",
                "position(base)[y]",
                "position(base)[z]",
                "attitude(base)[x]",
                "attitude(base)[y]",
                "attitude(base)[z]",
                "attitude(base)[w]",
            ];
            let id = result.new_writer(&body.body.get_name(), &celestial_folder_path, &headers);
            body.result_id = Some(id);
            // note i choose the names to be the same as multibody body so i can parse them easier for animation
            // if you change the names, animation wont work unless you change the column its looking for there            
        }

        let epoch_headers = ["sec_since_j2k", "julian_date"];
        let id = result.new_writer("epoch", &celestial_folder_path, &epoch_headers);
        self.epoch.result_id = Some(id);
    }

    pub fn write_results(
        &self,
        results: &mut ResultManager,
    ) {
        // update the epoch based on sim time
        if let Some(id) = &self.epoch.result_id {
            results.write_record(*id, &[
                self.epoch.time.get_seconds_j2k().to_string(),
                self.epoch.time.get_jd().to_string(),
            ]);
        }

        for body in &self.bodies {
            if let Some(id) = &body.result_id {
                results.write_record(
                    *id,
                    &[
                        body.position[0].to_string(),
                        body.position[1].to_string(),
                        body.position[2].to_string(),
                        body.orientation.x.to_string(),
                        body.orientation.y.to_string(),
                        body.orientation.z.to_string(),
                        body.orientation.s.to_string(),
                    ],
                );
            }
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
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct CelestialBody {
    pub body: CelestialBodies,
    pub position: Vector3<f64>,  // icrf
    pub orientation: Quaternion, // icrf to itrf
    pub gravity: Option<Gravity>,
    pub geomag: Option<GeoMagnetism>,
    pub result_id: Option<u32>,
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
            result_id: None,
        }
    }

    pub fn update(&mut self, t: Time, spice: &mut Spice) -> Result<(), CelestialErrors> {
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

    pub fn from_str(str: &str) -> Option<Self> {
        match str {
            "earth" => Some(CelestialBodies::Earth),
            "jupiter" => Some(CelestialBodies::Jupiter),
            "mars" => Some(CelestialBodies::Mars),
            "mercury" => Some(CelestialBodies::Mercury),
            "moon" => Some(CelestialBodies::Moon),
            "neptune" => Some(CelestialBodies::Neptune),
            "pluto" => Some(CelestialBodies::Pluto),
            "saturn" => Some(CelestialBodies::Saturn),
            "sun" => Some(CelestialBodies::Sun),
            "uranus" => Some(CelestialBodies::Uranus),
            "venus" => Some(CelestialBodies::Venus),
            _ => None,
        }
    }
}
