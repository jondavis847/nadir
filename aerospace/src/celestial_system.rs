use crate::{
    geomag::GeoMagnetism,
    gravity::{EGM96Gravity, Gravity, GravityTrait, NewtownianGravity},
};
use nalgebra::Vector3;
use polars::prelude::*;
use rotations::{
    prelude::{EulerAngles, EulerSequence, Quaternion},
    Rotation, RotationTrait,
};
use serde::{Deserialize, Serialize};
use spice::{Spice, SpiceBodies, SpiceErrors};
use std::f64::consts::PI;
use time::{Time, TimeErrors};

#[derive(Debug)]
pub enum CelestialErrors {
    BodyNotFoundInCelestialSystem,
    CelestialBodyAlreadyExists,
    InvalidEpoch,
    ResultConfigError,
    SpiceErrors(SpiceErrors),
    SpiceNotFound,
    TimeErrors(TimeErrors),
}

impl From<SpiceErrors> for CelestialErrors {
    fn from(value: SpiceErrors) -> Self {
        CelestialErrors::SpiceErrors(value)
    }
}

impl From<TimeErrors> for CelestialErrors {
    fn from(value: TimeErrors) -> Self {
        CelestialErrors::TimeErrors(value)
    }
}

#[derive(Clone, Debug, Deserialize, Serialize)]
pub struct CelestialSystem {
    pub epoch: Time,
    pub bodies: Vec<CelestialBody>,
}

impl CelestialSystem {
    pub fn new(epoch: Time) -> Result<Self, CelestialErrors> {
        Ok(Self {
            epoch,
            bodies: Vec::new(),
        })
    }

    pub fn update(&mut self, t: f64, spice: &mut Spice) -> Result<(), CelestialErrors> {
        // t is sim time in seconds
        let current_epoch = self.epoch + t;
        let current_epoch = current_epoch.to_system(time::TimeSystem::TT);
        for body in self.bodies.iter_mut() {
            body.update(current_epoch, spice)?;
        }
        Ok(())
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
        geomag: bool,
    ) -> Result<(), CelestialErrors> {
        // Check if the body already exists in the vector
        if self.bodies.iter().any(|b| b.body == body) {
            return Err(CelestialErrors::CelestialBodyAlreadyExists);
        }

        let gravity_option = if gravity {
            match body {
                CelestialBodies::Earth => Some(Gravity::EGM96(EGM96Gravity {})),
                CelestialBodies::Jupiter => Some(Gravity::Newtownian(NewtownianGravity::JUPITER)),
                CelestialBodies::Mars => Some(Gravity::Newtownian(NewtownianGravity::MARS)),
                CelestialBodies::Mercury => Some(Gravity::Newtownian(NewtownianGravity::MERCURY)),
                CelestialBodies::Moon => Some(Gravity::Newtownian(NewtownianGravity::MOON)),
                CelestialBodies::Neptune => Some(Gravity::Newtownian(NewtownianGravity::NEPTUNE)),
                CelestialBodies::Pluto => Some(Gravity::Newtownian(NewtownianGravity::PLUTO)),
                CelestialBodies::Saturn => Some(Gravity::Newtownian(NewtownianGravity::SATURN)),
                CelestialBodies::Sun => Some(Gravity::Newtownian(NewtownianGravity::SUN)),
                CelestialBodies::Venus => Some(Gravity::Newtownian(NewtownianGravity::VENUS)),
                CelestialBodies::Uranus => Some(Gravity::Newtownian(NewtownianGravity::URANUS)),
            }
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

    pub fn initialize_result(&self) -> CelestialResult {
        let mut body_results = Vec::new();
        for body in &self.bodies {
            body_results.push(CelestialBodyResult::new(body.body.clone()));
        }

        CelestialResult {
            epoch_sec_j2k_tai: Vec::new(),
            bodies: body_results,
        }
    }
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct CelestialBody {
    pub body: CelestialBodies,
    pub position: Vector3<f64>, // icrf
    pub orientation: Rotation,  // icrf to itrf
    pub gravity: Option<Gravity>,
    pub geomag: Option<GeoMagnetism>,
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
            orientation: Rotation::IDENTITY,
            gravity,
            geomag,
        }
    }

    pub fn update(&mut self, t: Time, spice: &mut Spice) -> Result<(), CelestialErrors> {
        //spice uses TDB/TT
        //gsfc planet fact sheet uses UTC
        //TODO: do these time calcs upstream and pass as args so we don't calculate for each body
        let utc = t.to_system(time::TimeSystem::UTC);
        let jdc = utc.get_jd_centuries();
        let sec_j2k = utc.get_seconds_j2k();

        self.position = spice.calculate_position(t, self.body.to_spice())?.into();

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
            _ => Rotation::IDENTITY, //TODO: how ot handle other bodies, warn and continue?
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
) -> Rotation {
    // j2000 orientation
    let ra = ra0 + ra1 * julian_centuries * PI / 180.0;
    let dec = dec0 + dec1 * julian_centuries * PI / 180.0;
    let initial_orientation = Rotation::from(EulerAngles::new(ra, dec, 0.0, EulerSequence::ZYX));
    // current orientation based on epoch
    let day = hrs_in_day * 3600.0;
    let rotation_rate = 2.0 * PI / day;
    let rotation = rotation_rate * sec_j2k;
    let orientation_from_rotation =
        Rotation::from(EulerAngles::new(rotation, 0.0, 0.0, EulerSequence::ZYX));
    orientation_from_rotation * initial_orientation
}

fn from_planet_fact_sheet_neptune(julian_centuries: f64, sec_j2k: f64) -> Rotation {
    // j2000 orientation
    let n = (357.85 + 52.316 * julian_centuries) * PI / 180.0;
    let ra = (299.36 + 0.70 * n.sin()) * PI / 180.0;
    let dec = (43.46 - 0.51 * n.cos()) * PI / 180.0;
    let initial_orientation = Rotation::from(EulerAngles::new(ra, dec, 0.0, EulerSequence::ZYX));
    // current orientation based on epoch
    let day = 16.11 * 3600.0;
    let rotation_rate = 2.0 * PI / day;
    let rotation = rotation_rate * sec_j2k;
    let orientation_from_rotation =
        Rotation::from(EulerAngles::new(rotation, 0.0, 0.0, EulerSequence::ZYX));
    orientation_from_rotation * initial_orientation
}

#[derive(Debug, Clone, Deserialize, Serialize)]
pub struct CelestialBodyResult {
    body: CelestialBodies,
    position: Vec<Vector3<f64>>,
    orientation: Vec<Quaternion>,
}

impl CelestialBodyResult {
    pub fn new(body: CelestialBodies) -> Self {
        Self {
            body,
            position: Vec::new(),
            orientation: Vec::new(),
        }
    }

    pub fn update(&mut self, body: &CelestialBody) {
        self.orientation.push(Quaternion::from(body.orientation));
        self.position.push(body.position);
    }

    pub fn add_to_dataframe(&self, name: &str, df: &mut DataFrame) {
        let orientation: Vec<Quaternion> = self
            .orientation
            .iter()
            .map(|rotation| Quaternion::from(*rotation))
            .collect();

        let orientation_x = Series::new(
            &format!("{name}_orientation_x"),
            orientation.iter().map(|v| v.x).collect::<Vec<f64>>(),
        );
        df.with_column(orientation_x).unwrap();

        let orientation_y = Series::new(
            &format!("{name}_orientation_y"),
            orientation.iter().map(|v| v.y).collect::<Vec<f64>>(),
        );
        df.with_column(orientation_y).unwrap();

        let orientation_z = Series::new(
            &format!("{name}_orientation_z"),
            orientation.iter().map(|v| v.z).collect::<Vec<f64>>(),
        );
        df.with_column(orientation_z).unwrap();

        let orientation_w = Series::new(
            &format!("{name}_orientation_w"),
            orientation.iter().map(|v| v.s).collect::<Vec<f64>>(),
        );
        df.with_column(orientation_w).unwrap();

        let body_position_x = Series::new(
            &format!("{name}_position_x"),
            self.position.iter().map(|v| v[0]).collect::<Vec<f64>>(),
        );
        df.with_column(body_position_x).unwrap();

        let body_position_y = Series::new(
            &format!("{name}_position_y"),
            self.position.iter().map(|v| v[1]).collect::<Vec<f64>>(),
        );
        df.with_column(body_position_y).unwrap();

        let body_position_z = Series::new(
            &format!("{name}_position_z"),
            self.position.iter().map(|v| v[2]).collect::<Vec<f64>>(),
        );
        df.with_column(body_position_z).unwrap();
    }
}

#[derive(Debug, Clone, Deserialize, PartialEq, Serialize)]
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
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CelestialResult {
    pub epoch_sec_j2k_tai: Vec<f64>,
    pub bodies: Vec<CelestialBodyResult>,
}

impl CelestialResult {
    pub fn update(&mut self, sys: &CelestialSystem) -> Result<(), CelestialErrors> {
        // it should be safe to do this enumerated since we initialized the result struct from the sys struct
        for i in 0..self.bodies.len() {
            let body_result = &mut self.bodies[i];
            let body = &sys.bodies[i];
            if body_result.body == body.body {
                body_result.update(body);
            } else {
                unreachable!("they should always be equal based on initialization")
            }
        }
        Ok(())
    }

    pub fn get_state_names(&self) -> Vec<String> {
        let mut names = Vec::new();
        names.push("epoch_sec_j2k_tai".to_string());

        for body_result in &self.bodies {
            let name = body_result.body.get_name();
            names.push(format!("{name}_orientation_x"));
            names.push(format!("{name}_orientation_y"));
            names.push(format!("{name}_orientation_z"));
            names.push(format!("{name}_orientation_w"));
            names.push(format!("{name}_position_x"));
            names.push(format!("{name}_position_y"));
            names.push(format!("{name}_position_z"));
        }
        names
    }

    pub fn add_to_dataframe(&self, df: &mut DataFrame) {
        let epoch_sec_j2k_tai = Series::new("epoch_sec_j2k_tai", self.epoch_sec_j2k_tai.clone());
        df.with_column(epoch_sec_j2k_tai).unwrap();

        for body_result in &self.bodies {
            let name = body_result.body.get_name();
            body_result.add_to_dataframe(&name, df);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use time::TimeSystem;

    #[test]
    fn test_earth_position_0() {         
        let mut spice = Spice::from_local().unwrap();
        let epoch = Time::from_sec_j2k(0.0, TimeSystem::UTC);
        let mut sys = CelestialSystem::new(epoch).unwrap();
        sys.add_body(CelestialBodies::Earth, true, false).unwrap();
        sys.update(0.0, &mut spice).unwrap();
        let result = &sys.bodies[0].position;

        assert_abs_diff_eq!(result[0], 0.0, epsilon = 1.0);
        assert_abs_diff_eq!(result[1], 0.0, epsilon = 1.0);
        assert_abs_diff_eq!(result[2], 0.0, epsilon = 1.0);
    }
}
