
use nalgebra::{Matrix3, Vector3};
use rotations::{rotation_matrix::RotationMatrix, RotationTrait};

pub const EARTH_MU: f64 = 3.986004418e14; // mu (m^3/s^2)
pub struct GravGradientTrq {
    //pub value: Vector3<f64>,
}
impl GravGradientTrq {
    pub fn calculate(
        &self,
        position: Vector3<f64>, // ECEF position
        q: &RotationMatrix,           // attitude i2b        
        a_f2i: &RotationMatrix,       // rotation ECEFToECI        
        moi: Matrix3<f64>,      // moment of inertia
        _mu: f64,                //Earth gravitational parameter, [m^3/s^2]
    ) -> Vector3<f64> {
        let mu = EARTH_MU;
        let n2 = mu / (position.norm()).powi(3); // square of orbital rate  [rad^2/s^2]        
        let a3: Vector3<f64> = -position / position.norm(); // LVLH Nadir
        let r_ecef_to_bcs = *q * *a_f2i;
        let a3_bcs: Vector3<f64> = r_ecef_to_bcs.inv().transform(a3);

        let gg_trq = 3.0 * n2 * a3_bcs.cross(&(moi * a3_bcs));
        gg_trq
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx_eq::assert_approx_eq;
    const TOL: f64 = 10e-12;
    use rotations::quaternion::Quaternion;

    #[test]
    fn test_gravity_gradient() {
        let g = GravGradientTrq{};
        let _mu = EARTH_MU;
        let position = Vector3::new(
            -821562.9892, 
            -906648.2064,
            -6954665.433); // from XINA (for PACE)
        let q = Quaternion::new(
            -0.053748871,
            -0.067546444,
            -0.994337304,
            -0.061982758); // from XINA (for PACE)
        let q_rot = RotationMatrix::from(q);
        
        let a_f2i = RotationMatrix::new(
            -0.80724874, -0.590206523, 0.002394065,
            0.590208289, -0.807250998, 0.0000388,
            0.001909682, 0.001444358, 0.999997133).unwrap(); // from XINA
        let moi:Matrix3<f64> = Matrix3::new(
            1.479651660912721e3,  -0.048887166933248e3,   0.034284737749261e3,
            -0.048887166933248e3,   1.384248902142130e3,  -0.089444552930578e3,
            0.034284737749261e3,  -0.089444552930578e3,   2.296663349501903e3,); 
        let mu = EARTH_MU;

        let gg = g.calculate(position, &q_rot, &a_f2i.into(), moi, mu);
        let expected_gg: nalgebra::Matrix<f64, nalgebra::Const<3>, nalgebra::Const<1>, nalgebra::ArrayStorage<f64, 3, 1>>= Vector3::new(
            0.872625864581703e-3,
            -0.595283109273388e-3,
            -0.116261767084494e-3); // get this from Matlab
        
        assert_approx_eq!(gg.x, expected_gg.x);
        assert_approx_eq!(gg.y,expected_gg.y);
        assert_approx_eq!(gg.z,expected_gg.z);
    }

}
