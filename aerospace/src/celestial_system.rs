use crate::{
    earth::{Earth, EarthResult},
    gravity::{Gravity, GravityTrait},
};
use nalgebra::Vector3;
use polars::prelude::*;
use rotations::{prelude::Quaternion, Rotation, RotationTrait};
use serde::{Deserialize, Serialize};
use spice::{Spice, SpiceBodies, SpiceErrors};
use time::{Time, TimeErrors};

#[derive(Debug)]
pub enum CelestialErrors {
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

    pub fn add_earth(&mut self, earth: Earth) -> Result<(), CelestialErrors> {
        match self.bodies.earth {
            Some(_) => return Err(CelestialErrors::CelestialBodyAlreadyExists),
            None => self.bodies.earth = Some(earth),
        }
        Ok(())
    }

    pub fn delete_earth(&mut self) {
        self.bodies.earth = None;
    }

    pub fn add_moon(&mut self) -> Result<(), CelestialErrors> {
        match self.bodies.moon {
            Some(_) => return Err(CelestialErrors::CelestialBodyAlreadyExists),
            None => self.bodies.moon = Some(CelestialBody::default()),
        }
        Ok(())
    }

    pub fn delete_moon(&mut self) {
        self.bodies.moon = None;
    }

    pub fn add_sun(&mut self) -> Result<(), CelestialErrors> {
        match self.bodies.sun {
            Some(_) => return Err(CelestialErrors::CelestialBodyAlreadyExists),
            None => self.bodies.sun = Some(CelestialBody::default()),
        }
        Ok(())
    }

    pub fn delete_sun(&mut self) {
        self.bodies.sun = None;
    }

    /// Calculates gravity in the itrf frame based on position in the itrf frame
    pub fn calculate_gravity_itrf(&self, position: Vector3<f64>) -> Vector3<f64> {
        let mut g = Vector3::zeros();
        if let Some(earth) = &self.bodies.earth {
            if let Some(gravity) = &earth.gravity {
                g = gravity.calculate(position);
            }
        }
        g
    }

    /// Calculates gravity in the j2000 ecliptic frame based on position in the j2000 ecliptic frame
    pub fn calculate_gravity_gcrf(&self, position: Vector3<f64>) -> Vector3<f64> {
        let mut g = Vector3::zeros();
        if let Some(earth) = &self.bodies.earth {
            if let Some(gravity) = &earth.gravity {
                // rotate is just inverse of transform, and we want itrf_from_gcrf
                // TODO: Store position_ecef in body state?
                let position_ecef = earth.gcrf_from_itrf.rotate(position);
                g = self.calculate_gravity_itrf(position_ecef);
                match gravity {
                    Gravity::EGM96(_) => {
                        //comes out in itrf (ecef), convert to gcrf ecliptic (eci)
                        g = earth.gcrf_from_itrf.transform(g);
                    }
                    _ => {}
                }
            }
        }
        g
    }

    pub fn initialize_result(&self) -> CelestialResult {
        let earth = match self.bodies.earth {
            Some(_) => Some(EarthResult::default()),
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
    pub orientation: Rotation, // icrf to itrf
    pub gravity: Option<Gravity>,
    pub geomag: Option<GeoMagnetism>,
}

impl CelestialBody {
    pub fn update(
        &mut self,
        t: Time,
        spice: &mut Spice,
        body: SpiceBodies,
    ) -> Result<(), CelestialErrors> {
        let position = spice.calculate_position(t, body)?;
        self.position_icrf = position.into();
        self.orientation_icrf = Rotation::IDENTITY; //todo add more orientations if necessary
        Ok(())
    }
}

#[derive(Debug, Default, Clone, Deserialize, Serialize)]
pub struct CelestialBodyResult {
    position_icrf: Vec<Vector3<f64>>,
    orientation_icrf: Vec<Rotation>,
}

impl CelestialBodyResult {
    pub fn update(&mut self, body: &CelestialBody) {
        self.orientation_icrf.push(body.orientation_icrf);
        self.position_icrf.push(body.position_icrf);
    }

    pub fn add_to_dataframe(&self, name: &str, df: &mut DataFrame) {
        let orientation: Vec<Quaternion> = self.orientation_icrf.iter().map(|rotation| Quaternion::from(*rotation)).collect();
        
        let orientation_x = Series::new(
            &format!("{name}_orientation_x"),
            orientation
                .iter()
                .map(|v| v.x)
                .collect::<Vec<f64>>(),
        );
        df.with_column(orientation_x).unwrap();

        let orientation_y = Series::new(
            &format!("{name}_orientation_y"),
            orientation
                .iter()
                .map(|v| v.y)
                .collect::<Vec<f64>>(),
        );
        df.with_column(orientation_y).unwrap();

        let orientation_z = Series::new(
            &format!("{name}_orientation_z"),
            orientation
                .iter()
                .map(|v| v.z)
                .collect::<Vec<f64>>(),
        );
        df.with_column(orientation_z).unwrap();

        let orientation_w = Series::new(
            &format!("{name}_orientation_w"),
            orientation
                .iter()
                .map(|v| v.s)
                .collect::<Vec<f64>>(),
        );
        df.with_column(orientation_w).unwrap();

        let body_position_x = Series::new(
            &format!("{name}_position_x"),
            self.position_icrf
                .iter()
                .map(|v| v[0])
                .collect::<Vec<f64>>(),
        );
        df.with_column(body_position_x).unwrap();

        let body_position_y = Series::new(
            &format!("{name}_position_y"),
            self.position_icrf
                .iter()
                .map(|v| v[1])
                .collect::<Vec<f64>>(),
        );
        df.with_column(body_position_y).unwrap();

        let body_position_z = Series::new(
            &format!("{name}_position_z"),
            self.position_icrf
                .iter()
                .map(|v| v[2])
                .collect::<Vec<f64>>(),
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
    pub earth: Option<Earth>,
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
            body.update(t, spice)?;
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
    pub earth: Option<EarthResult>,
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
        if let (Some(earth), Some(earth_result)) = (&sys.bodies.earth, &mut self.earth) {
            earth_result.gcrf_from_itrf.push(earth.gcrf_from_itrf);
            earth_result.position_gcrf.push(earth.position_gcrf);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.jupiter, &mut self.jupiter) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.mars, &mut self.mars) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.mercury, &mut self.mercury) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.moon, &mut self.moon) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.neptune, &mut self.neptune) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.pluto, &mut self.pluto) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.saturn, &mut self.saturn) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.sun, &mut self.sun) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.uranus, &mut self.uranus) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        if let (Some(body), Some(body_result)) = (&sys.bodies.venus, &mut self.venus) {
            body_result.update(body);
        } else {
            return Err(CelestialErrors::ResultConfigError);
        }

        Ok(())
    }

    fn get_state_names(&self) -> Vec<String> {}
    fn get_result_entry(&self) -> ResultEntry {}

    fn add_to_dataframe(&self, df: &mut DataFrame) {
        let epoch_sec_j2k_tai = Series::new("epoch_sec_j2k_tai", self.epoch_sec_j2k_tai);
        df.with_column(epoch_sec_j2k_tai).unwrap();

        if let Some(earth) = &self.earth {
            let earth_orientation_x = Series::new(
                "earth_gcrf_from_itrf_x",
                earth
                    .gcrf_from_itrf
                    .iter()
                    .map(|v| v.x)
                    .collect::<Vec<f64>>(),
            );
            df.with_column(earth_orientation_x).unwrap();

            let earth_orientation_y = Series::new(
                "earth_gcrf_from_itrf_y",
                earth
                    .gcrf_from_itrf
                    .iter()
                    .map(|v| v.y)
                    .collect::<Vec<f64>>(),
            );
            df.with_column(earth_orientation_y).unwrap();

            let earth_orientation_z = Series::new(
                "earth_gcrf_from_itrf_z",
                earth
                    .gcrf_from_itrf
                    .iter()
                    .map(|v| v.z)
                    .collect::<Vec<f64>>(),
            );
            df.with_column(earth_orientation_z).unwrap();

            let earth_orientation_w = Series::new(
                "earth_gcrf_from_itrf_w",
                earth
                    .gcrf_from_itrf
                    .iter()
                    .map(|v| v.s)
                    .collect::<Vec<f64>>(),
            );
            df.with_column(earth_orientation_w).unwrap();

            let earth_position_x = Series::new(
                "earth_position_x",
                earth
                    .position_gcrf
                    .iter()
                    .map(|v| v[0])
                    .collect::<Vec<f64>>(),
            );
            df.with_column(earth_orientation_x).unwrap();

            let earth_position_y = Series::new(
                "earth_position_y",
                earth
                    .position_gcrf
                    .iter()
                    .map(|v| v[1])
                    .collect::<Vec<f64>>(),
            );
            df.with_column(earth_orientation_y).unwrap();

            let earth_position_z = Series::new(
                "earth_position_z",
                earth
                    .position_gcrf
                    .iter()
                    .map(|v| v[2])
                    .collect::<Vec<f64>>(),
            );
            df.with_column(earth_orientation_z).unwrap();
        }
        /*

        if let Some(body) = &self.jupiter {
            body.add_to_dataframe("jupiter", df)
        }
        if let Some(body) = &self.mars {
            body.add_to_dataframe("mars", df)
        }
        */
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use time::TimeSystem;

    #[test]
    fn test_gravity_egm96() {
        let mut spice = Spice::from_local().unwrap();
        let epoch = Time::from_sec_j2k(0.0, TimeSystem::UTC);
        let mut sys = CelestialSystem::new(epoch).unwrap();
        let earth = Earth::default();
        sys.add_earth(earth).unwrap();
        sys.update(0.0, &mut spice).unwrap();
        let earth = &sys.bodies.earth.clone().unwrap();
        let gcrf_from_itrf = earth.gcrf_from_itrf.clone();
        let itrf_from_gcrf = gcrf_from_itrf.inv();

        //note gcrf is gcrf ecliptic, not equatorial
        let position_gcrf = Vector3::new(1.0e7, 0.0, 0.0);
        let position_itrf = itrf_from_gcrf.transform(position_gcrf);
        assert_abs_diff_eq!(position_itrf[0], 1.815851240993843e6, epsilon = 1.0);
        assert_abs_diff_eq!(position_itrf[1], 9.833752296057563e6, epsilon = 1.0);
        assert_abs_diff_eq!(position_itrf[2], -251.83481732831436, epsilon = 1.0);
        let g_itrf = sys.calculate_gravity_itrf(position_itrf);
        assert_abs_diff_eq!(g_itrf[0], -0.7242743501831314, epsilon = 1e-3);
        assert_abs_diff_eq!(g_itrf[1], -3.922275259999543, epsilon = 1e-3);
        assert_abs_diff_eq!(g_itrf[2], 9.280404810605218e-5, epsilon = 1e-3);
    }
}
