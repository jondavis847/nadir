//use core::time;
//use std::vec;
//use std::{f32::consts::PI, sync::Arc, time::SystemTime};
//use std::f64::consts::PI;
use interp::{interp, InterpMode};
use nalgebra::Vector3;
use serde::{Serialize, Deserialize};
//use crate::gravity::EARTH_RE;
//pub const MU: f64 = 3.986004418e14;
// temp assignment for const coefficients: 
pub const DRAG_COEFF: f64 = 1.15; // shape of a short cylinder
pub const SMA: f64 = 1.0; // semi major axis
pub const ECCENTRICITY: f64 = 0.5; // eccentricity
pub const AREA: f64 = 15.0; // area
pub const OFFSET_Y: f64 = 1.0; // cp-cg (y offsett)
pub const OFFSET_Z: f64 = 1.0; // cp-cg (z offsett)

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct AeroDynamicsDrag {
}

impl AeroDynamicsDrag{
   pub fn calculate(&self, velocity: Vector3<f64>, height:f64) -> Vector3<f64> {
    /*
    let ecc = ECCENTRICITY;
    //let t:f64 = self.clone_from(&time);
    //let t:f64 = 0.01;
    let n:f64 = (MU/SMA.powi(3)).sqrt();
    let p = SMA*(1.0-ecc.powi(2));
    let capital_m:f64 = n*t;
    let capital_e:f64 = find_capital_e(capital_m, ecc);
    let tru_anomaly:f64 = 2.0*(((1.0+ecc)/(1.0-ecc)).sqrt()*(capital_e/2.0).tan()).atan();
    let r =  p/(1.0+ecc*tru_anomaly.cos());
    //let v:f64 = MU*(2.0/r - 1.0/MU).sqrt();
    let h = r - EARTH_RE;
    */
    let v:f64 = velocity.magnitude(); // velocity magnitude 
    let density = atm_density(height);
    let force_air_drag:f64 = 0.5*(density*DRAG_COEFF*AREA*v.powi(2));
    let tq_z:f64 = force_air_drag * OFFSET_Y;
    let tq_y:f64 = force_air_drag * OFFSET_Z;
    let tq_air_drag:Vector3<f64> = Vector3::new(0.0, tq_y, tq_z);

    dbg!(tq_air_drag);

    /*fn find_capital_e (capital_m:f64,eccentricity:f64)-> f64{
        let pi:f64 = PI as f64;
        //let mut capital_e:f64 = 0.0;
        if capital_m > -pi && capital_m < 0.0 || capital_m > pi {
            capital_e = capital_m - eccentricity;
        } else {
            capital_e = capital_m + eccentricity;
        }
        capital_e
    }*/

    fn atm_density (h_in:f64) -> f64 {
        let height: Vec<f64> = Vec::from([100.0,200.0,300.0,400.0,500.0,600.0,
            700.0,800.0,900.0,1000.0]);
        let rho:Vec<f64> = Vec::from(
            [5.603998064021879e-07,2.540582078157406e-10,1.915693553323112e-11,
            2.802563293290757e-12,5.215070519051597e-13,1.136639043398080e-13,
            3.069405743845089e-14, 1.135772369932094e-14,5.758938544921416e-15,
            3.560576449319286e-15]);
        let atm_density:f64 = interp(&height, &rho, h_in, &InterpMode::Extrapolate);
        atm_density
    }
    tq_air_drag
   }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    const TOL: f64 = 1e-12;

    #[test]
    fn test_air_drag_trq() {
        let trq = AeroDynamicsDrag{};
        let velocity = Vector3::new(
            -7405.819636, 
            931.4468463,
            753.7540218); // from XINA (for PACE)
        let height:f64 = 670.0;

        let air_drag = trq.calculate(velocity, height);
        let expected_air_drag = Vector3::new(
            0.0,
            0.269827106195429e-4,
            0.269827106195429e-4); // get this from Matlab

        assert_abs_diff_eq!(air_drag.x,expected_air_drag.x, epsilon = TOL);
        assert_abs_diff_eq!(air_drag.y,expected_air_drag.y, epsilon = TOL);
        assert_abs_diff_eq!(air_drag.z,expected_air_drag.z, epsilon = TOL);

    }

}