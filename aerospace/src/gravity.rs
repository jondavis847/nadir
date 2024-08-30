use nalgebra::{Matrix2, SimdBool, Vector3}; //DMatrix
use std::f64::consts::PI;
use serde::{Serialize, Deserialize};

pub const EARTH: f64 = 3.986004418e14; // mu (m^3/s^2)
// pub const EARTH_J2: f64 = 1082e-6;
pub const EARTH_RE: f64 = 6378137.0; // (m)  TODO: implement WGS84
pub const MAX_DEG: u8 = 2; //16; // degrees of spherical harmonics
//pub const EGM96_C:Vec<Vec<f64>> = (e11=-1.01,  0.86],[3.98,  0.53]];
//pub const EGM96_S:Vec<Vec<f64>> = Matrix2::new(-1.01,  0.86, 3.98,  0.53);
pub const EGM96_C: Matrix2<f64> = Matrix2::new(-1.01, 0.86, 3.98, 0.53);
pub const EGM96_S: Matrix2<f64> = Matrix2::new(-1.01, 0.86, 3.98, 0.53);

//pub const EGM96_C2:DMatrix<f64> = DMatrix::from_data(C);
/*pub fn coeff(){
    let _file = File::open(&"egm96.mat").expect("Unable to open file");
    let mat_file = matfile::MatFile::parse(_file).unwrap();
    let egm_c3::DMatrix = mat_file.find_by_name("test");
    //let C = FILE.dataset("C").unwrap().read().unwrap();
//pub const C: DMatrix<f64> = FILE.dataset("C").unwrap().read().unwrap();
}
const FILE:File = File::open(&"egm96.mat").expect("Unable to open file");
pub const C: File = FILE.dataset("C").unwrap().read().unwrap();
//pub const C: DMatrix<f64> = FILE.dataset("C").unwrap().read().unwrap();
pub const EGM96_C2:DMatrix<f64> = DMatrix::from_data(C);

/*pub const EGM96_C:Vec<Vec<f64>> = array![
    [-1.01,  0.86],
    [ 3.98,  0.53]];*/

/* //pub const FILE_PATH: String =  r"C:\Users\mbakhtia\PACE Repo\pace-gnc-spare2\Hifi\models\LIB\Environment\egm96.mat";
//pub const file: std::fs::File::open("C:\Users\mbakhtia\PACE Repo\pace-gnc-spare2\Hifi\models\LIB\Environment\egm96.mat")?;
//pub const FILE::File::open(FILE_PATH);
//pub const mat_file: = matfile::MatFile::parse(FILE)?; */
*/

pub const MOON: f64 = 4.9048695e12;

pub const SUN: f64 = 1.32712440018e20;

pub const MERCURY: f64 = 2.2032e13;

pub const VENUS: f64 = 3.24859e14;

pub const MARS: f64 = 4.282837e13;

pub const JUPITER: f64 = 1.26686534e17;

pub const SATURN: f64 = 3.7931187e16;

pub const URANUS: f64 = 5.793939e15;

pub const NEPTUNE: f64 = 6.836529e15;

pub const PLUTO: f64 = 8.71e11;

pub trait GravityTrait {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64>;
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum Gravity {
    Constant(ConstantGravity),
    TwoBody(TwoBodyGravity),
    // EGM96(J2Harmonic),
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
pub struct TwoBodyGravity {
    pub mu: f64,
}

impl TwoBodyGravity {
    pub const EARTH: Self = Self { mu: EARTH };

    pub const MOON: Self = Self { mu: MOON };

    pub const SUN: Self = Self { mu: SUN };

    pub const MERCURY: Self = Self { mu: MERCURY };

    pub const VENUS: Self = Self { mu: VENUS };

    pub const MARS: Self = Self { mu: MARS };

    pub const JUPITER: Self = Self { mu: JUPITER };

    pub const SATURN: Self = Self { mu: SATURN };

    pub const URANUS: Self = Self { mu: URANUS };

    pub const NEPTUNE: Self = Self { mu: NEPTUNE };

    pub const PLUTO: Self = Self { mu: PLUTO };

    pub fn new(mu: f64) -> Self {
        Self { mu }
    }
}

#[derive(Debug, Clone, serde::Serialize, serde::Deserialize)]
pub struct EGM96Gravity {
    pub mu: f64, // is this neccessary?
    pub earth_re: f64,
    pub maxdeg: u8,
    pub c: Matrix2<f64>, // Vec<Vec<f64>>, // Matrix2<f64>,
    pub s: Matrix2<f64>, // Vec<Vec<f64>>, // Matrix2<f64>,
}
impl EGM96Gravity {
    pub const EGM96_COEFF: Self = Self {
        mu: EARTH,
        earth_re: EARTH_RE,
        maxdeg: MAX_DEG,
        c: EGM96_C,
        s: EGM96_S,
    };
}
/* pub struct J2Harmonic {
    pub mu: f64, // is this neccessary?
    pub j2: f64,
}
impl J2Harmonic {
    pub const EARTH_J2: Self = Self {mu: EARTH, j2: EARTH_J2};
} */
impl GravityTrait for ConstantGravity {
    fn calculate(&self, _position: Vector3<f64>) -> Vector3<f64> {
        self.value
    }
}

impl GravityTrait for TwoBodyGravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        let position_mag = position.magnitude();
        -position * self.mu / position_mag.powi(3) // point mass two body model
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
        let n = self.maxdeg as usize;
        let sm_lambda: Vec<f64> = lambda_coeff(n, lambda).0;
        let cm_lambda: Vec<f64> = lambda_coeff(n, lambda).1;
        let p_coeff: Vec<Vec<f64>> = legendre_func(lat, n).0;
        let scale_factor: Vec<Vec<f64>> = legendre_func(lat, n).1;
        let g_ecef: Vector3<f64> = loc_gravity_ecef(
            position,
            n,
            p_coeff,
            self.c,
            self.s,
            sm_lambda,
            cm_lambda,
            self.mu,
            self.earth_re,
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
            p[1][1] = 1.0; // n = 0, m = 0
            p[2][1] = (3f64).sqrt() * cphi; // n = 1, m = 0
            scale_factor[1][1] = 0.0;
            scale_factor[2][1] = 1.0;
            p[2][2] = (3f64).sqrt() * sphi; // n = 1, m = 1
            scale_factor[2][2] = 0.0;

            for n in 2..=maxdeg + 2 {
                let k = n + 1;
                for m in 0..=n {
                    let p_index = m + 1;
                    // Compute normalized associated legendre polynomials, P, via recursion relations
                    // Scale Factor needed for normalization of dUdphi partial derivative
                    if n == m {
                        p[k][k] = ((2.0 * n as f64 + 1.0).sqrt() / (2.0 * n as f64).sqrt())
                            * sphi
                            * p[k - 1][k - 1];
                        scale_factor[k][k] = 0.0;
                    } else if m == 0 {
                        p[k][p_index] = ((2.0 * n as f64 + 1.0) / n as f64)
                            * ((2.0 * n as f64 - 1.0).sqrt() * cphi * p[k - 1][p_index]
                                - (n as f64 - 1.0) / ((2.0 * n as f64 - 3.0).sqrt())
                                    * p[k - 2][p_index]);
                        scale_factor[k][p_index] = ((n as f64 + 1.0) * n as f64 / 2.0).sqrt();
                    } else {
                        p[k][p_index] = ((2.0 * n as f64 + 1.0)
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
            c: Matrix2<f64>,
            s: Matrix2<f64>,
            smlambda: Vec<f64>,
            cmlambda: Vec<f64>,
            mu: f64,
            re: f64, // earth radius
            r: f64,  // position_mag
            scale_factor: Vec<Vec<f64>>,
        ) -> Vector3<f64> {
            let r_ratio: f64 = re / r;
            let mut r_ratio_n = r_ratio;

            // Initialize summation of gravity in radial coordinates
            let mut du_dr_sum_n = 1.0;
            let mut du_dphi_sum_n = 0.0;
            let mut du_dlambda_sum_n = 0.0;

            // Summation of gravity in radial coordinates
            for n in 2..=maxdeg {
                let k = n + 1;
                r_ratio_n *= r_ratio;
                let mut du_dr_sum_m = 0.0;
                let mut du_dphi_sum_m = 0.0;
                let mut du_dlambda_sum_m = 0.0;

                for m in 0..=n {
                    let j = m + 1;
                    du_dr_sum_m += p[k][j] * (c[(k, j)] * cmlambda[j] + s[(k, j)] * smlambda[j]);
                    du_dphi_sum_m += p[k][j + 1] * scale_factor[k][j]
                        - pos[2] / (pos[0].powi(2) + pos[1].powi(2))
                            * m as f64
                            * p[k][j]
                            * (c[(k, j)] * cmlambda[j] + s[(k, j)] * smlambda[j]);
                    du_dlambda_sum_m +=
                        m as f64 * p[k][j] * (s[(k, j)] * cmlambda[j] - c[(k, j)] * smlambda[j]);
                }
                du_dr_sum_n += du_dr_sum_m * r_ratio_n * k as f64;
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
            let g = Vector3::new(gx, gy, gz);
            g
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

/* impl GravityTrait for J2Harmonic {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        let position_mag = position.magnitude();
        let lat: f64 = (position.z/position_mag).asin();
        // gravity with J2 harmonics:
        -self.mu* position / position_mag.powi(3) + // spherical term - point mass two body model
        3.0*self.mu*self.j2* EARTH_RE.powi(2) * position * position_mag.powi(-5) * (-0.5 + 1.5 * lat.sin().powi(2)) // non-spherical - J2 term
    }
} */

impl GravityTrait for Gravity {
    fn calculate(&self, position: Vector3<f64>) -> Vector3<f64> {
        match self {
            Gravity::Constant(gravity) => gravity.calculate(position),
            Gravity::TwoBody(gravity) => gravity.calculate(position),
            Gravity::EGM96(gravity) => gravity.calculate(position),
        }
    }
}

/* #[cfg(test)]
mod tests {
    use super::*;
    use approx_eq::assert_approx_eq;
    use std::f64::consts::PI;
    const TOL: f64 = 1e-12;

    /// Unittest for EGM96 (J2 Harmonics).
    #[test]
    fn test_j2_harmonics() {
        let p = Vector3::


        let q = Quaternion::new(1.0, 2.0, 3.0, 4.0).normalize();

        assert_approx_eq!(q.x, 0.18257418583505536, TOL);
    }

    /// Test for quaternion inversion.
    #[test]
    fn test_quaternion_inv() {
        let quat = Quaternion::rand();
        let inv = quat.inv();

        assert_approx_eq!(inv.s, quat.s);
        assert_approx_eq!(inv.x, -quat.x);
    }
} */
