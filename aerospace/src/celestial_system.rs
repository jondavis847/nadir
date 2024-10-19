use crate::{
    geomag::GeoMagnetism,
    gravity::{EGM96Gravity, Gravity, GravityTrait, TwoBodyGravity},
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
    pub bodies: CelestialBodiesCache,
}

impl CelestialSystem {
    pub fn new(epoch: Time) -> Result<Self, CelestialErrors> {
        Ok(Self {
            epoch,
            bodies: CelestialBodiesCache::default(),
        })
    }

    pub fn update(&mut self, t: f64, spice: &mut Spice) -> Result<(), CelestialErrors> {
        // t is sim time in seconds
        let current_epoch = self.epoch + t;
        let current_epoch = current_epoch.to_system(time::TimeSystem::TT);
        self.bodies.update(current_epoch, spice)?;
        Ok(())
    }

    pub fn set_geomag(
        &mut self,
        body: CelestialBodies,
        b: Option<GeoMagnetism>,
    ) -> Result<(), CelestialErrors> {
        let set_body_geomag = |body_option: &mut Option<CelestialBody>| {
            if let Some(body) = body_option {
                body.geomag = b;
                Ok(())
            } else {
                Err(CelestialErrors::BodyNotFoundInCelestialSystem)
            }
        };

        match body {
            CelestialBodies::Earth => set_body_geomag(&mut self.bodies.earth),
            CelestialBodies::Jupiter => set_body_geomag(&mut self.bodies.jupiter),
            CelestialBodies::Mars => set_body_geomag(&mut self.bodies.mars),
            CelestialBodies::Mercury => set_body_geomag(&mut self.bodies.mercury),
            CelestialBodies::Moon => set_body_geomag(&mut self.bodies.moon),
            CelestialBodies::Neptune => set_body_geomag(&mut self.bodies.neptune),
            CelestialBodies::Pluto => set_body_geomag(&mut self.bodies.pluto),
            CelestialBodies::Saturn => set_body_geomag(&mut self.bodies.saturn),
            CelestialBodies::Sun => set_body_geomag(&mut self.bodies.sun),
            CelestialBodies::Uranus => set_body_geomag(&mut self.bodies.uranus),
            CelestialBodies::Venus => set_body_geomag(&mut self.bodies.venus),
        }
    }

    pub fn set_gravity(
        &mut self,
        body: CelestialBodies,
        g: Option<Gravity>,
    ) -> Result<(), CelestialErrors> {
        let set_body_gravity = |body_option: &mut Option<CelestialBody>| {
            if let Some(body) = body_option {
                body.gravity = g;
                Ok(())
            } else {
                Err(CelestialErrors::BodyNotFoundInCelestialSystem)
            }
        };

        match body {
            CelestialBodies::Earth => set_body_gravity(&mut self.bodies.earth),
            CelestialBodies::Jupiter => set_body_gravity(&mut self.bodies.jupiter),
            CelestialBodies::Mars => set_body_gravity(&mut self.bodies.mars),
            CelestialBodies::Mercury => set_body_gravity(&mut self.bodies.mercury),
            CelestialBodies::Moon => set_body_gravity(&mut self.bodies.moon),
            CelestialBodies::Neptune => set_body_gravity(&mut self.bodies.neptune),
            CelestialBodies::Pluto => set_body_gravity(&mut self.bodies.pluto),
            CelestialBodies::Saturn => set_body_gravity(&mut self.bodies.saturn),
            CelestialBodies::Sun => set_body_gravity(&mut self.bodies.sun),
            CelestialBodies::Uranus => set_body_gravity(&mut self.bodies.uranus),
            CelestialBodies::Venus => set_body_gravity(&mut self.bodies.venus),
        }
    }

    pub fn add_body(
        &mut self,
        body: CelestialBodies,
        gravity: bool,
        geomag: bool,
    ) -> Result<(), CelestialErrors> {
        match body {
            CelestialBodies::Earth => {
                if self.bodies.earth.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::EGM96(EGM96Gravity {}))
                    } else {
                        None
                    };
                    self.bodies.earth = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Jupiter => {
                if self.bodies.jupiter.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::JUPITER))
                    } else {
                        None
                    };
                    self.bodies.jupiter = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Mars => {
                if self.bodies.mars.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::MARS))
                    } else {
                        None
                    };
                    self.bodies.mars = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Mercury => {
                if self.bodies.mercury.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::MERCURY))
                    } else {
                        None
                    };
                    self.bodies.mercury = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Moon => {
                if self.bodies.moon.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::MOON))
                    } else {
                        None
                    };
                    self.bodies.moon = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Neptune => {
                if self.bodies.neptune.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::NEPTUNE))
                    } else {
                        None
                    };
                    self.bodies.neptune = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Pluto => {
                if self.bodies.pluto.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::PLUTO))
                    } else {
                        None
                    };
                    self.bodies.pluto = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Saturn => {
                if self.bodies.saturn.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::SATURN))
                    } else {
                        None
                    };
                    self.bodies.saturn = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Sun => {
                if self.bodies.sun.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::SUN))
                    } else {
                        None
                    };
                    self.bodies.sun = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Venus => {
                if self.bodies.venus.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::VENUS))
                    } else {
                        None
                    };
                    self.bodies.venus = Some(CelestialBody::new(gravity, None));
                }
            }
            CelestialBodies::Uranus => {
                if self.bodies.uranus.is_some() {
                    return Err(CelestialErrors::CelestialBodyAlreadyExists);
                } else {
                    let gravity = if gravity {
                        Some(Gravity::TwoBody(TwoBodyGravity::URANUS))
                    } else {
                        None
                    };
                    self.bodies.uranus = Some(CelestialBody::new(gravity, None));
                }
            }
        }
        Ok(())
    }

    pub fn delete_body(&mut self, body: CelestialBodies) {
        match body {
            CelestialBodies::Earth => self.bodies.earth = None,
            CelestialBodies::Jupiter => self.bodies.jupiter = None,
            CelestialBodies::Mars => self.bodies.mars = None,
            CelestialBodies::Mercury => self.bodies.mercury = None,
            CelestialBodies::Moon => self.bodies.moon = None,
            CelestialBodies::Neptune => self.bodies.neptune = None,
            CelestialBodies::Pluto => self.bodies.pluto = None,
            CelestialBodies::Saturn => self.bodies.saturn = None,
            CelestialBodies::Sun => self.bodies.sun = None,
            CelestialBodies::Venus => self.bodies.venus = None,
            CelestialBodies::Uranus => self.bodies.uranus = None,
        }
    }

    /// calculates gravity based on all bodies in the celestial system with a gravity model
    /// position is in the gcrf/j2000 frame
    pub fn calculate_gravity(&self, position: Vector3<f64>) -> Vector3<f64> {
        let mut g_final = Vector3::zeros();

        if let Some(earth) = &self.bodies.earth {
            if let Some(gravity) = &earth.gravity {
                match gravity {
                    Gravity::EGM96(gravity) => {                        
                        let position_itrf = earth.orientation.rotate(position);
                        let g_itrf = gravity.calculate(position_itrf);
                        let g_gcrf = earth.orientation.transform(g_itrf);
                        g_final += g_gcrf;
                    }
                    Gravity::TwoBody(gravity) => g_final += gravity.calculate(position),
                    Gravity::Constant(gravity) => g_final += gravity.calculate(position),
                }                               
            }
        }
        if let Some(body) = &self.bodies.jupiter {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        if let Some(body) = &self.bodies.mars {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        if let Some(body) = &self.bodies.mercury {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        if let Some(body) = &self.bodies.moon {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        if let Some(body) = &self.bodies.neptune {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        if let Some(body) = &self.bodies.pluto {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        if let Some(body) = &self.bodies.saturn {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        if let Some(body) = &self.bodies.sun {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        if let Some(body) = &self.bodies.uranus {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        if let Some(body) = &self.bodies.venus {
            if let Some(gravity) = &body.gravity {
                g_final += gravity.calculate(position);
            }
        }

        g_final
    }

    pub fn initialize_result(&self) -> CelestialResult {
        let earth = match self.bodies.earth {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let jupiter = match self.bodies.jupiter {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let mars = match self.bodies.mars {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let mercury = match self.bodies.mercury {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let moon = match self.bodies.moon {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let neptune = match self.bodies.neptune {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let pluto = match self.bodies.pluto {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let saturn = match self.bodies.saturn {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let sun = match self.bodies.sun {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let uranus = match self.bodies.uranus {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };
        let venus = match self.bodies.venus {
            Some(_) => Some(CelestialBodyResult::default()),
            None => None,
        };

        CelestialResult {
            epoch_sec_j2k_tai: Vec::new(),
            earth,
            jupiter,
            mercury,
            mars,
            moon,
            neptune,
            pluto,
            saturn,
            sun,
            uranus,
            venus,
        }
    }
}

#[derive(Debug, Default, Clone, Deserialize, Serialize)]
pub struct CelestialBody {
    pub position: Vector3<f64>, // icrf
    pub orientation: Rotation,  // icrf to itrf
    pub gravity: Option<Gravity>,
    pub geomag: Option<GeoMagnetism>,
}

impl CelestialBody {
    pub fn new(gravity: Option<Gravity>, geomag: Option<GeoMagnetism>) -> Self {
        Self {
            gravity,
            geomag,
            ..Default::default()
        }
    }

    pub fn update(
        &mut self,
        t: Time,
        spice: &mut Spice,
        body: SpiceBodies,
    ) -> Result<(), CelestialErrors> {
        //spice uses TDB/TT
        //gsfc planet fact sheet uses UTC
        //TODO: do these time calcs upstream and pass as args so we don't calculate for each body
        let utc = t.to_system(time::TimeSystem::UTC);
        let jdc = utc.get_jd_centuries();
        let sec_j2k = utc.get_seconds_j2k();

        self.position = spice.calculate_position(t, body)?.into();

        self.orientation = match body {
            SpiceBodies::Earth | SpiceBodies::Moon => spice.calculate_orientation(t, body)?,
            SpiceBodies::Jupiter => {
                from_planet_fact_sheet(268.057, -0.006, 64.495, 0.002, 9.9250, jdc, sec_j2k)
            }
            SpiceBodies::Mars => {
                from_planet_fact_sheet(317.681, -0.106, 52.887, -0.061, 24.6229, jdc, sec_j2k)
            }
            SpiceBodies::Mercury => {
                from_planet_fact_sheet(281.01, -0.033, 61.414, -0.005, 1407.6, jdc, sec_j2k)
            }
            SpiceBodies::Neptune => from_planet_fact_sheet_neptune(jdc, sec_j2k),
            SpiceBodies::Saturn => {
                from_planet_fact_sheet(40.589, -0.036, 83.537, -0.004, 10.656, jdc, sec_j2k)
            }
            SpiceBodies::Uranus => {
                from_planet_fact_sheet(257.311, 0.0, -15.175, 0.0, -17.24, jdc, sec_j2k)
            }
            SpiceBodies::Venus => {
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

#[derive(Debug, Default, Clone, Deserialize, Serialize)]
pub struct CelestialBodyResult {
    position: Vec<Vector3<f64>>,
    orientation: Vec<Quaternion>,
}

impl CelestialBodyResult {
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

#[derive(Debug, Clone, Deserialize, Serialize)]
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

#[derive(Default, Debug, Clone, Deserialize, Serialize)]
pub struct CelestialBodiesCache {
    pub earth: Option<CelestialBody>,
    jupiter: Option<CelestialBody>,
    mercury: Option<CelestialBody>,
    mars: Option<CelestialBody>,
    moon: Option<CelestialBody>,
    neptune: Option<CelestialBody>,
    pluto: Option<CelestialBody>,
    saturn: Option<CelestialBody>,
    sun: Option<CelestialBody>,
    uranus: Option<CelestialBody>,
    venus: Option<CelestialBody>,
}

impl CelestialBodiesCache {
    pub fn update(&mut self, t: Time, spice: &mut Spice) -> Result<(), CelestialErrors> {
        if let Some(body) = &mut self.earth {
            body.update(t, spice, SpiceBodies::Earth)?;
        }
        if let Some(body) = &mut self.jupiter {
            body.update(t, spice, SpiceBodies::Jupiter)?;
        }

        if let Some(body) = &mut self.mercury {
            body.update(t, spice, SpiceBodies::Mercury)?;
        }

        if let Some(body) = &mut self.mars {
            body.update(t, spice, SpiceBodies::Mars)?;
        }

        if let Some(body) = &mut self.moon {
            body.update(t, spice, SpiceBodies::Moon)?;
        }

        if let Some(body) = &mut self.neptune {
            body.update(t, spice, SpiceBodies::Neptune)?;
        }

        if let Some(body) = &mut self.pluto {
            body.update(t, spice, SpiceBodies::Pluto)?;
        }

        if let Some(body) = &mut self.saturn {
            body.update(t, spice, SpiceBodies::Saturn)?;
        }

        if let Some(body) = &mut self.sun {
            body.update(t, spice, SpiceBodies::Sun)?;
        }

        if let Some(body) = &mut self.uranus {
            body.update(t, spice, SpiceBodies::Uranus)?;
        }

        if let Some(body) = &mut self.venus {
            body.update(t, spice, SpiceBodies::Venus)?;
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct CelestialResult {
    pub epoch_sec_j2k_tai: Vec<f64>,
    pub earth: Option<CelestialBodyResult>,
    pub jupiter: Option<CelestialBodyResult>,
    pub mercury: Option<CelestialBodyResult>,
    pub mars: Option<CelestialBodyResult>,
    pub moon: Option<CelestialBodyResult>,
    pub neptune: Option<CelestialBodyResult>,
    pub pluto: Option<CelestialBodyResult>,
    pub saturn: Option<CelestialBodyResult>,
    pub sun: Option<CelestialBodyResult>,
    pub uranus: Option<CelestialBodyResult>,
    pub venus: Option<CelestialBodyResult>,
}

impl CelestialResult {
    pub fn update(&mut self, sys: &CelestialSystem) -> Result<(), CelestialErrors> {
        let mut bodies = [
            (&sys.bodies.earth, &mut self.earth),
            (&sys.bodies.jupiter, &mut self.jupiter),
            (&sys.bodies.mars, &mut self.mars),
            (&sys.bodies.mercury, &mut self.mercury),
            (&sys.bodies.moon, &mut self.moon),
            (&sys.bodies.neptune, &mut self.neptune),
            (&sys.bodies.pluto, &mut self.pluto),
            (&sys.bodies.saturn, &mut self.saturn),
            (&sys.bodies.sun, &mut self.sun),
            (&sys.bodies.uranus, &mut self.uranus),
            (&sys.bodies.venus, &mut self.venus),
        ];

        for (body, body_result) in bodies.iter_mut() {
            if let (Some(body), Some(body_result)) = (body, body_result) {
                body_result.update(body);
            } else {
                return Err(CelestialErrors::ResultConfigError);
            }
        }

        Ok(())
    }

    pub fn get_state_names(&self) -> Vec<String> {
        let mut names = Vec::new();

        // Create an array of tuples representing celestial bodies and their names
        let bodies = [
            (&self.earth, "earth"),
            (&self.jupiter, "jupiter"),
            (&self.mars, "mars"),
            (&self.mercury, "mercury"),
            (&self.moon, "moon"),
            (&self.neptune, "neptune"),
            (&self.pluto, "pluto"),
            (&self.sun, "sun"),
            (&self.uranus, "uranus"),
            (&self.venus, "venus"),
        ];

        // Iterate over the array and add each body's data to the DataFrame
        for (body_option, name) in bodies.iter() {
            if body_option.is_some() {
                names.push(format!("{name}_orientation_x"));
                names.push(format!("{name}_orientation_y"));
                names.push(format!("{name}_orientation_z"));
                names.push(format!("{name}_orientation_w"));
                names.push(format!("{name}_position_x"));
                names.push(format!("{name}_position_y"));
                names.push(format!("{name}_position_z"));
            }
        }
        names
    }

    pub fn add_to_dataframe(&self, df: &mut DataFrame) {
        let epoch_sec_j2k_tai = Series::new("epoch_sec_j2k_tai", self.epoch_sec_j2k_tai.clone());
        df.with_column(epoch_sec_j2k_tai).unwrap();

        // Create an array of tuples representing celestial bodies and their names
        let bodies = [
            (&self.earth, "earth"),
            (&self.jupiter, "jupiter"),
            (&self.mars, "mars"),
            (&self.mercury, "mercury"),
            (&self.moon, "moon"),
            (&self.neptune, "neptune"),
            (&self.pluto, "pluto"),
            (&self.sun, "sun"),
            (&self.uranus, "uranus"),
            (&self.venus, "venus"),
        ];

        // Iterate over the array and add each body's data to the DataFrame
        for (body_option, name) in bodies.iter() {
            if let Some(body) = body_option {
                body.add_to_dataframe(name, df);
            }
        }
    }
}

// #[cfg(test)]
// mod tests {
//     use super::*;
//     use approx::assert_abs_diff_eq;
//     use time::TimeSystem;

//     // #[test]
//     // fn test_gravity_egm96() {
//     //     let mut spice = Spice::from_local().unwrap();
//     //     let epoch = Time::from_sec_j2k(0.0, TimeSystem::UTC);
//     //     let mut sys = CelestialSystem::new(epoch).unwrap();        
//     //     sys.add_body(CelestialBodies::Earth, true, false);
//     //     sys.update(0.0, &mut spice).unwrap();
//     //     let earth = &sys.bodies.earth.clone().unwrap();
//     //     let gcrf_from_itrf = earth.orientation.clone();
//     //     let itrf_from_gcrf = gcrf_from_itrf.inv();

        
//     // }
// }
