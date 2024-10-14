use crate::{
    earth::Earth,
    gravity::{Gravity, GravityTrait},
};
use nalgebra::Vector3;
use rotations::{Rotation, RotationTrait};
use serde::{Deserialize, Serialize};
use spice::{Spice, SpiceBodies, SpiceErrors};
use time::{Time, TimeErrors};

#[derive(Debug)]
pub enum CelestialErrors {
    CelestialBodyAlreadyExists,
    InvalidEpoch,
    SpiceErrors(SpiceErrors),
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
    #[serde(skip_serializing)]
    pub spice: Spice,    
}

impl CelestialSystem {
    pub fn new(epoch: Time) -> Result<Self, CelestialErrors> {
        Ok(Self {
            epoch,
            bodies: CelestialBodiesCache::default(),
            spice: Spice::from_naif()?,
        })
    }

    pub fn update(&mut self, t: f64) -> Result<(), CelestialErrors> {
        // t is sim time in seconds
        let current_epoch = self.epoch + t;
        let current_epoch = current_epoch.to_system(time::TimeSystem::TT);        
        self.bodies.update(current_epoch, &mut self.spice)?;
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
                        dbg!(&g);
                    }
                    _ => {}
                }
            }
        }
        g
    }
}

#[derive(Debug, Default, Clone, Deserialize, Serialize)]
pub struct CelestialBody {
    position_icrf: Vector3<f64>,
    orientation_icrf: Rotation,
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

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use time::TimeSystem;

    #[test]
    fn test_gravity_egm96() {
        let epoch = Time::from_sec_j2k(0.0, TimeSystem::UTC);
        let mut sys = CelestialSystem::new(epoch).unwrap();
        let earth = Earth::default();
        sys.add_earth(earth).unwrap();
        sys.update(0.0).unwrap();
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
        assert_abs_diff_eq!(g_itrf[2],  9.280404810605218e-5, epsilon = 1e-3);
    }
}
