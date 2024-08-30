
use nalgebra::{Matrix3, Vector3};
use rotations::{Rotation, RotationTrait, quaternion::Quaternion, rotation_matrix::RotationMatrix};

pub const EARTH_MU: f64 = 3.986004418e14; // mu (m^3/s^2)
pub struct GravGradientTrq {
    pub value: Vector3<f64>,
}
impl GravGradientTrq {
    fn calculate(
        &self,
        position: Vector3<f64>, // ECEF position
        q: &Rotation,           // attitude i2b
        a_f2i: &Rotation,       // rotation ECEFToECI
        moi: Matrix3<f64>,      // moment of inertia
        mu: f64,                //Earth gravitational parameter, [m^3/s^2]
    ) -> Vector3<f64> {
        let mu = EARTH_MU;
        let n2 = mu / (position.norm()).powi(3); // square of orbital rate  [rad^2/s^2]
        let a3: Vector3<f64> = -position / position.norm(); // LVLH Nadir

        //let q: Quaternion<f64> = [q[0], q[1], q[2], q[3]];
        //let qtoa:Matrix3<f64> = Force::from(Vector6::new(

        let r_ecef_to_bcs = *q * *a_f2i;
        let a3_bcs: Vector3<f64> = r_ecef_to_bcs.transform(a3);
        let gg_trq = 3.0 * n2 * a3_bcs.cross(&(moi * a3_bcs));
        gg_trq
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn test_gravity_gradient() {
        let g = GravGradientTrq{};
        let mu = EARTH_MU;
        let position = [-821562.9892, -906648.2064, -6954665.433];
        let q = Quaternion::new(
            -0.053748871,
            -0.067546444,
            -0.994337304,
            -0.061982758);
            
        let a_f2i = Matrix3::new(
            -0.80724874, -0.590206523, 0.002394065,
            0.590208289, -0.807250998, 3.88E-05,
            0.001909682, 0.001444358, 0.999997133); 
        let moi:Matrix3<f64> = Matrix3::new(
            461.650, -128.0900, -5.6240,
            -128.0900, 586.3, -34.6280,
            -5.6240, -34.6280, 583.5); 
        let mu = EARTH_MU;

/*        let qi2b = Rotation::from(Quaternion::new(
            -0.053748871,
            -0.067546444,
            -0.994337304,
            -0.061982758));
*/
/*         let a_f2i = Rotation::from(RotationMatrix::new(
            -0.80724874, -0.590206523, 0.002394065,
            0.590208289, -0.807250998, 3.88E-05,
            0.001909682, 0.001444358, 0.999997133)); 
*/


        let gg = g.calculate(position, q, a_f2i, moi, mu);
        let expected_gg= Vector3::new(1.0, 2.0, 3.0);

        assert_eq!(gg, expected_gg);
    }

}
