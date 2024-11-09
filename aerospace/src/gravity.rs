use nalgebra::{Matrix5, SimdBool, Vector3}; //DMatrix
use serde::{Deserialize, Serialize};
use std::f64::consts::PI;

use crate::celestial_system::CelestialBodies;

pub const MAX_DEG: u8 = 4; //16; // degrees of spherical harmonics

pub const EGM96_C: Matrix5<f64> = Matrix5::new(
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    -0.000484165371736000,
    -0.000000000186987636,
    0.000002439143523980,
    0.0,
    0.0,
    0.000000957254173792,
    0.000002029988821840,
    0.000000904627768605,
    0.000000721072657057,
    0.0,
    0.000000539873863789,
    -0.000000536321616971,
    0.000000350694105785,
    0.000000990771803829,
    -0.000000188560802735,
);

pub const EGM96_S: Matrix5<f64> = Matrix5::new(
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.0,
    0.000119528012031e-5,
    -0.140016683654000e-5,
    0.0,
    0.0,
    0.0,
    0.024851315871600e-5,
    -0.061902594420500e-5,
    0.141435626958000e-5,
    0.0,
    0.0,
    -0.047344026585300e-5,
    0.066267157254000e-5,
    -0.020092836917700e-5,
    0.030885316933300e-5,
);

pub trait GravityTrait {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64>;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Gravity {
    Constant(ConstantGravity),
    Newtownian(NewtownianGravity),
    EGM96(EGM96Gravity),
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ConstantGravity {
    pub value: Vector3<f64>,
}
impl ConstantGravity {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self {
            value: Vector3::new(x, y, z),
        }
    }
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NewtownianGravity {
    pub mu: f64,
}

impl NewtownianGravity {
    pub fn new(mu: f64) -> Self {
        Self { mu }
    }

    pub fn from_body(body: CelestialBodies) -> Self {
        Self { mu: body.get_mu() }
    }
}

impl GravityTrait for NewtownianGravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        let position_mag = position.magnitude();
        if position_mag < 0.1 {
            println!("WARNING! division by zero on two body gravity!");
        }
        -position * self.mu / position_mag.powi(3) // point mass two body model
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EGM96Gravity {}
impl GravityTrait for ConstantGravity {
    fn calculate(&self, _position: Vector3<f64>) -> Vector3<f64> {
        self.value
    }
}

impl GravityTrait for EGM96Gravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        let px = position.x;
        let py = position.y;
        let pz = position.z;
        let position_mag = position.magnitude();
        let lat: f64 = (pz / position_mag).asin(); // latitude (rad)
        let lambda: f64 = py.atan2(px); // longitude (rad)
        let n = MAX_DEG as usize; //self.deg as usize;
        let sm_lambda: Vec<f64> = lambda_coeff(n, lambda).0;
        let cm_lambda: Vec<f64> = lambda_coeff(n, lambda).1;
        let p_coeff: Vec<Vec<f64>> = legendre_func(lat, n).0;
        let scale_factor: Vec<Vec<f64>> = legendre_func(lat, n).1;
        let g_ecef: Vector3<f64> = loc_gravity_ecef(
            position,
            n,
            p_coeff,
            EGM96_C, //self.c,
            EGM96_S, //self.s,
            sm_lambda,
            cm_lambda,
            position_mag,
            scale_factor,
        );

        fn legendre_func(phi: f64, maxdeg: usize) -> (Vec<Vec<f64>>, Vec<Vec<f64>>) {
            let mut p = vec![vec![0.0; maxdeg + 3]; maxdeg + 3];
            let mut scale_factor = vec![vec![0.0; maxdeg + 3]; maxdeg + 3];
            let mut cphi: f64 = (PI / 2.0 - phi).cos();
            let mut sphi: f64 = (PI / 2.0 - phi).sin();
            // Force numerically zero values to be exactly zero
            if cphi.abs() <= f64::EPSILON {
                cphi = 0.0;
            }
            if sphi.abs() <= f64::EPSILON {
                sphi = 0.0;
            }

            // Seeds for recursion formula
            p[0][0] = 1.0; // n = 0, m = 0
            p[1][0] = (3f64).sqrt() * cphi; // n = 1, m = 0
            scale_factor[0][0] = 0.0;
            scale_factor[1][0] = 1.0;
            p[1][1] = (3f64).sqrt() * sphi; // n = 1, m = 1
            scale_factor[1][1] = 0.0;

            for n in 2..=maxdeg + 2 {
                //            for n in 2..=maxdeg + 1 {
                //              let k = n + 1;
                let k = n;
                for m in 0..=n {
                    //let p_index = m + 1;
                    let p_index = m;
                    // Compute normalized associated legendre polynomials, P, via recursion relations
                    // Scale Factor needed for normalization of dUdphi partial derivative
                    if n == m {
                        p[k][k] = ((2.0 * n as f64 + 1.0).sqrt() / (2.0 * n as f64).sqrt())
                            * sphi
                            * p[k - 1][k - 1];
                        scale_factor[k][k] = 0.0;
                    } else if m == 0 {
                        p[k][p_index] = ((2.0 * n as f64 + 1.0).sqrt() / n as f64)
                            * ((2.0 * n as f64 - 1.0).sqrt() * cphi * p[k - 1][p_index]
                                - (n as f64 - 1.0) / ((2.0 * n as f64 - 3.0).sqrt())
                                    * p[k - 2][p_index]);
                        scale_factor[k][p_index] = ((n as f64 + 1.0) * n as f64 / 2.0).sqrt();
                    } else {
                        p[k][p_index] = ((2.0 * n as f64 + 1.0).sqrt()
                            / ((n + m) as f64).sqrt()
                            / ((n - m) as f64).sqrt())
                            * ((2.0 * n as f64 - 1.0).sqrt() * cphi * p[k - 1][p_index]
                                - ((n + m - 1) as f64).sqrt() * ((n - m - 1) as f64).sqrt()
                                    / ((2.0 * n as f64 - 3.0).sqrt())
                                    * p[k - 2][p_index]);
                        scale_factor[k][p_index] = ((n + m + 1) as f64 * (n - m) as f64).sqrt();
                    }
                }
            }
            (p, scale_factor)
        }

        fn loc_gravity_ecef(
            pos: Vector3<f64>, // position
            maxdeg: usize,
            p: Vec<Vec<f64>>,
            c: Matrix5<f64>,
            s: Matrix5<f64>,
            smlambda: Vec<f64>,
            cmlambda: Vec<f64>,
            r: f64, // position_mag
            scale_factor: Vec<Vec<f64>>,
        ) -> Vector3<f64> {
            let earth = CelestialBodies::Earth;
            let re = earth.get_radius();
            let mu = earth.get_mu();

            let r_ratio = re / r;
            let mut r_ratio_n = r_ratio;

            // Initialize summation of gravity in radial coordinates
            let mut du_dr_sum_n = 1.0;
            let mut du_dphi_sum_n = 0.0;
            let mut du_dlambda_sum_n = 0.0;

            // Summation of gravity in radial coordinates
            for n in 2..=maxdeg {
                //let k = n + 1;
                let k = n;
                r_ratio_n *= r_ratio;
                let mut du_dr_sum_m = 0.0;
                let mut du_dphi_sum_m = 0.0;
                let mut du_dlambda_sum_m = 0.0;

                for m in 0..=n {
                    //let j = m + 1;
                    let j = m;
                    du_dr_sum_m += p[k][j] * (c[(k, j)] * cmlambda[j] + s[(k, j)] * smlambda[j]);
                    du_dphi_sum_m += (p[k][j + 1] * scale_factor[k][j]
                        - pos[2] / (pos[0].powi(2) + pos[1].powi(2)).sqrt() * m as f64 * p[k][j])
                        * (c[(k, j)] * cmlambda[j] + s[(k, j)] * smlambda[j]);
                    du_dlambda_sum_m +=
                        m as f64 * p[k][j] * (s[(k, j)] * cmlambda[j] - c[(k, j)] * smlambda[j]);
                }
                du_dr_sum_n += du_dr_sum_m * r_ratio_n * (k as f64 + 1.0);
                du_dphi_sum_n += du_dphi_sum_m * r_ratio_n;
                du_dlambda_sum_n += du_dlambda_sum_m * r_ratio_n;
            }

            // Gravity in spherical coordinates
            let du_dr = -mu / r.powi(2) * du_dr_sum_n;
            let du_dphi = mu / r * du_dphi_sum_n;
            let du_dlambda = mu / r * du_dlambda_sum_n;

            // Gravity in ECEF coordinates
            let mut gx = (1.0 / r * du_dr
                - pos[2] / (r.powi(2) * (pos[0].powi(2) + pos[1].powi(2)).sqrt()) * du_dphi)
                * pos[0]
                - (du_dlambda / (pos[0].powi(2) + pos[1].powi(2))) * pos[1];
            let mut gy = (1.0 / r * du_dr
                - pos[2] / (r.powi(2) * (pos[0].powi(2) + pos[1].powi(2)).sqrt()) * du_dphi)
                * pos[1]
                + (du_dlambda / (pos[0].powi(2) + pos[1].powi(2))) * pos[0];
            let mut gz = 1.0 / r * du_dr * pos[2]
                + ((pos[0].powi(2) + pos[1].powi(2)).sqrt() / r.powi(2)) * du_dphi;

            // Special case for poles
            let at_pole = (pos[2].atan2((pos[0].powi(2) + pos[1].powi(2)).sqrt())).abs()
                == std::f64::consts::PI / 2.0;
            if at_pole.any() {
                gx = 0.0;
                gy = 0.0;
                gz = 1.0 / r * du_dr * pos[2];
            }
            Vector3::new(gx, gy, gz)            
        }

        fn lambda_coeff(max_deg: usize, lambda: f64) -> (Vec<f64>, Vec<f64>) {
            //let num_rows = pos_ecef.len();
            let mut sm_lambda = vec![0.0; max_deg + 1];
            let mut cm_lambda = vec![0.0; max_deg + 1];

            let slambda = lambda.sin();
            let clambda = lambda.cos();
            sm_lambda[0] = 0.0;
            cm_lambda[0] = 1.0;
            sm_lambda[1] = slambda;
            cm_lambda[1] = clambda;
            for m in 2..=max_deg {
                sm_lambda[m] = 2.0 * clambda * sm_lambda[m - 1] - sm_lambda[m - 2];
                cm_lambda[m] = 2.0 * clambda * cm_lambda[m - 1] - cm_lambda[m - 2];
            }

            (sm_lambda, cm_lambda)
        }
        g_ecef
    }
}

impl GravityTrait for Gravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        match self {
            Gravity::Constant(gravity) => gravity.calculate(position),
            Gravity::Newtownian(gravity) => gravity.calculate(position),
            Gravity::EGM96(gravity) => gravity.calculate(position),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Unittest for EGM96
    #[test]
    fn grav_egm96() {
        let grav = EGM96Gravity {};
        let pos_ecef = Vector3::new(-821562.9892, -906648.2064, -6954665.433);

        let g_rust = grav.calculate(pos_ecef);
        let g_pace_model = Vector3::new(0.925349412278864, 1.021116998885220, 7.853405068561626);

        assert_relative_eq!(g_rust.x, g_pace_model.x, max_relative = 1e-3);
        assert_relative_eq!(g_rust.y, g_pace_model.y, max_relative = 1e-3);
        assert_relative_eq!(g_rust.z, g_pace_model.z, max_relative = 1e-3);
    }
}
