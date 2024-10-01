
use nalgebra::Vector3;
use serde::{Serialize, Deserialize};
use std::f64::consts::PI;

pub const AREA_SA: f64 = 11.19632; // PACE Model
pub const OFFSET_SA: Vector3<f64> = Vector3::new(
    0.007020132559606,  
    -2.684757719935861,
    -0.321759875628063); // cg(sa)-cg(total) (based on PACE model)
pub const OFFSET_BUS: Vector3<f64> = Vector3::new(
    -0.000478561655160,
    0.183019634921002,
    0.021934334905677); // cg(bus)-cg(total) (based on PACE model)
pub const REF_SPEC_SA: f64 = 0.8; // sepcular reflectivity for SA (based on PACE model)
pub const REF_DFFS_SA: f64 = 0.2; // diffuse reflectivity for SA (based on PACE model)
pub const REF_SPEC_BUS: f64 = 0.8; // sepcular reflectivity for BUS (based on PACE model)
pub const REF_DFFS_BUS: f64 = 0.2; // diffuse reflectivity for BUS (based on PACE model)
pub const SOLAR_CONSTANT: f64 = 1361.0; // w/m2 solar maximum
pub const LIGHT_SPEED: f64 = 299792458.0; // m/s

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SolarRadiationPressure {
}

impl SolarRadiationPressure{
   pub fn calculate(&self, sc_sun_unit_v:Vector3<f64>, sa_norm:Vector3<f64>, height:f64, dia:f64 ) 
   -> Vector3<f64> {
    
    let h = height; // bus height
    let d = dia; // bus cylinder diameter
    let bus_norm:Vector3<f64> = Vector3::new(-1.0, 0.0, 0.0); // body normal (based on PACE model)
    let cd_sa:f64 = REF_DFFS_SA;
    let cs_sa:f64 = REF_SPEC_SA;
    let cd_bus:f64 = REF_DFFS_BUS;
    let cs_bus:f64 = REF_SPEC_BUS;
    let flux:f64 = SOLAR_CONSTANT;
    let c:f64 = LIGHT_SPEED;
    let cos_theta:f64 = sc_sun_unit_v.dot(&sa_norm); // cosine of angle of incidence
    let a = (sc_sun_unit_v.dot(&bus_norm)).acos(); // angle b/w bus symmetry axis and scTosunUnitV
    
    let force_srp_sa:Vector3<f64> = -flux/c * AREA_SA * cos_theta * 
        ((1.0-cs_sa)*sc_sun_unit_v + 2.0*(cd_sa/3.0 + cs_sa*cos_theta)*sa_norm); // SA SRP Force 
    let force_srp_bus:Vector3<f64> = -flux/c * (((((1.0+cs_bus/3.0)*a.sin() + PI/6.0*cd_bus)*d*h) +
        ((1.0-cs_bus)*PI/4.0*d.powi(2)*a.cos()))*sc_sun_unit_v +
        (((cd_bus/3.0+cs_bus*a.cos())*2.0*a.cos()*PI/4.0*d.powi(2)) +
        (-4.0/3.0*cs_bus*a.sin() - PI/6.0*cd_bus)*a.cos()*d*h)*bus_norm); // Bus SRP Force

        dbg!((((1.0+cs_bus/3.0)*a.sin() + PI/6.0*cd_bus)*d*h) + ((1.0-cs_bus)*PI/4.0*d.powi(2)*a.cos()));
        dbg!(((cd_bus/3.0+cs_bus*a.cos())*2.0*a.cos()*PI/4.0*d.powi(2)) + (-4.0/3.0*cs_bus*a.sin() - PI/6.0*cd_bus)*a.cos()*d*h);

    let tq_srp_sa:Vector3<f64> = OFFSET_SA.cross(&force_srp_sa); // SRP torque due to SA
    let tq_srp_bus:Vector3<f64> = OFFSET_BUS.cross(&force_srp_bus); // SRP torque due to BUS

    tq_srp_sa + tq_srp_bus    

}
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx_eq::assert_approx_eq;
    const TOL: f64 = 1e-12;

    #[test]
    fn test_srp_trq() {
        let trq = SolarRadiationPressure{};
        let sa_normal:Vector3<f64>= Vector3::new(
            0.0,  
            -0.235142113102590,
            -0.971961000578546); // from PACE simulation
        let sc_sun_unit_v:Vector3<f64>= Vector3::new(
            0.356510722456053,
            0.278555703053872,
            -0.891799767363743); // from PACE simulation
        let bus_height:f64 = 3.459; // PACE model
        let bus_diameter:f64 = 1.469; // PACE model


        let srp = trq.calculate(sc_sun_unit_v, sa_normal, bus_height, bus_diameter);
        let expected_srp_tq = Vector3::new(
            1.0e-03 * -0.161364749238317,     
            1.0e-03 * 0.000499395817770,
            1.0e-03 * -0.007687592190709); // (bus+sa) from PACE simulation
        /*let expected_srp_force = Vector3::new(
            1.0e-04 * -0.030462325224874,
            1.0e-04 * 0.031619391202898,
            1.0e-04 * 0.893069031908440); // (bus+sa) from PACE simulation
        let expected_srp_sa_force = Vector3::new(
            1.0e-04 * -0.029040723629221,
            1.0e-04 * 0.112864433180066,
            1.0e-04 * 0.632961955512723); // (sa) from PACE simulation
        let expected_srp_bus_force = Vector3::new(
            1.0e-04 * -0.001421601595652,
            1.0e-04 * -0.081245041977168,
            1.0e-04 * 0.260107076395717); // (sa) from PACE simulation
    */

                             
        assert_approx_eq!(srp.x , expected_srp_tq.x, TOL);
        assert_approx_eq!(srp.y , expected_srp_tq.y, TOL);
        assert_approx_eq!(srp.z , expected_srp_tq.z, TOL);

    }

}