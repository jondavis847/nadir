use rotations::prelude::Quaternion;
use rotations::{Rotation, RotationTrait};
use spice::{Spice,SpiceBodies};

use crate::gravity::EGM96Gravity;
use crate::{celestial_system::CelestialErrors, gravity::Gravity};
use crate::geomag::GeoMagnetism;
use nalgebra::Vector3;
use serde::{Deserialize,Serialize};
use time::Time;

#[derive(Debug,Clone,Deserialize,Serialize)]
pub struct Earth {
    pub position_gcrf: Vector3<f64>,
    pub gcrf_from_itrf: Rotation, 
    pub gravity: Option<Gravity>,
    pub geomag: Option<GeoMagnetism>,    
    gcrf_ecliptic_from_equatorial: Rotation,
}

impl Default for Earth {    
    fn default() -> Self {
        let gravity = Some(Gravity::EGM96(EGM96Gravity{}));
        let geomag = None;

        //TODO: include century calculation for obliquity for completeness?
        let obliquity = 23.43928111111111 * std::f64::consts::PI / 180.0; // https://ssd.jpl.nasa.gov/astro_par.html
        let gcrf_ecliptic_from_equatorial = Rotation::from(Quaternion::new(
            (obliquity/2.0).sin(), 0.0,0.0,(obliquity/2.0).cos()
        ));
        Self {
            position_gcrf: Vector3::zeros(),
            gcrf_from_itrf: Rotation::IDENTITY,
            gravity,
            geomag,            
            gcrf_ecliptic_from_equatorial
        }
    }
}

impl Earth {    
    pub fn update(&mut self, t: Time, spice: &mut Spice) -> Result<(),CelestialErrors> {        
        let position = spice.calculate_position(t, SpiceBodies::Earth)?;        
        self.position_gcrf = position.into();
        let orientation = spice.calculate_orientation(t, SpiceBodies::Earth)?;        
        //orientation is the active rotation from ecliptic of j2000 to itrf
        //convert to an active rotation from equatorial j2000 to itrf        
        let itrf_from_ecliptic = orientation.0;
        let itrf_from_equatorial = itrf_from_ecliptic * self.gcrf_ecliptic_from_equatorial;        
        self.gcrf_from_itrf = itrf_from_equatorial.inv();
        Ok(())
    }    
}

#[derive(Debug,Default,Clone,Deserialize,Serialize)]
pub struct EarthResult {
    pub position_gcrf: Vec<Vector3<f64>>,
    pub gcrf_from_itrf: Vec<Rotation>,
    pub gcrf_from_icrf: Vec<Rotation>,
}

#[cfg(test)]
    mod tests {
        use super::*;
        use approx::assert_abs_diff_eq;
        use time::TimeSystem;

        #[test]
        fn test_earth_gcrf_from_itrf() {            
            let mut earth = Earth::default();                        
            let mut spice = Spice::from_naif().unwrap();

            earth.update(Time::from_sec_j2k(0.0,TimeSystem::TT),&mut spice).unwrap();
            let gcrf_from_itrf = Quaternion::from(earth.gcrf_from_itrf);

            //These are set loose for now since the orientation can change with new files
            //comparison values pulled from naif mice matlab cspice interface 'cspice_pxform('J2000','ITRF93',0.0)'
            assert_abs_diff_eq!(gcrf_from_itrf.x, 1.86188367e-5, epsilon = 1e-6);
            assert_abs_diff_eq!(gcrf_from_itrf.y, -8.46884056e-7, epsilon = 1e-6);
            assert_abs_diff_eq!(gcrf_from_itrf.z, -0.6414902205, epsilon = 1e-6);            
            // this comes out positive from spice, but is passive rotation, mine are active and allow neg scalars, so negative
            assert_abs_diff_eq!(gcrf_from_itrf.s, -0.76713121214, epsilon = 1e-6);  


            earth.update(Time::from_sec_j2k(1e6,TimeSystem::TT),&mut spice).unwrap();
            let gcrf_from_itrf = Quaternion::from(earth.gcrf_from_itrf);

            //These are set loose for now since the orientation can change with new files
            //comparison values pulled from naif mice matlab cspice interface 'cspice_pxform('J2000','ITRF93',0.0)'
            assert_abs_diff_eq!(gcrf_from_itrf.x, 6.20496e-6, epsilon = 1e-6);
            assert_abs_diff_eq!(gcrf_from_itrf.y, 1.74531e-5, epsilon = 1e-6);
            assert_abs_diff_eq!(gcrf_from_itrf.z, -0.934416, epsilon = 1e-6);            
            // this comes out positive from spice, but is passive rotation, mine are active and allow neg scalars, so negative
            assert_abs_diff_eq!(gcrf_from_itrf.s, 0.356183, epsilon = 1e-6);            
        }
    }