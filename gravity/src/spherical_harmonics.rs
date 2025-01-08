use std::f64::consts::PI;

use nalgebra::Vector3;

use crate::legendre::Legendre;

#[derive(Debug)]
pub struct SphericalHarmonics {
    order: usize,
    legendre: Legendre,
}

impl SphericalHarmonics {
    pub fn new(degree:usize, order: usize) -> Self {
        Self {            
            order,
            legendre: Legendre::new(degree, order).with_derivatives(),
        }
    }

    /// returns the gradient of V
    pub fn vallado(
        &mut self,
        r_ecef: Vector3<f64>,
        c: &Vec<Vec<f64>>,
        s: &Vec<Vec<f64>>,
        re: f64,
        mu: f64,
    ) -> Vector3<f64> {
        let x = r_ecef.x;
        let y = r_ecef.y;
        let z = r_ecef.z;
        let r = r_ecef.norm();
        let xy = (x * x + y * y).sqrt();
        let rr = re/r;
        //TODO: Put these in a separate function?
        let latgc = (z/r).asin();
        let lon = {
            let lon = if xy < 1e-9 {
                y.signum() * PI * 0.5
            } else {
                y.atan2(x)
            };
            let lon = lon % (2.0 * PI);
            lon
        };

        self.legendre.calculate(latgc.sin());

        let mut partial_r = 0.0;
        let mut partial_lat = 0.0;
        let mut partial_lon = 0.0;

        let p = &self.legendre.p;

        for l in 2..=self.order {
            let rrl = rr.powi(l as i32);
            for m in 0..=l {
                let clm = (m as f64 * lon).cos();
                let slm = (m as f64 * lon).sin();

                partial_r +=  rrl * (l as f64 + 1.0) * p[l][m] * (c[l][m] * clm + s[l][m] * slm);
                partial_lat += rrl * (p[l][m + 1] - m as f64 * latgc.tan() * p[l][m])
                    * (c[l][m] * clm + s[l][m] * slm);
                partial_lon += rrl * m as f64 * p[l][m] * (s[l][m] * clm - c[l][m] * slm);
            }
        }

        partial_r *= -mu / r.powi(2);
        partial_lat *= mu / r;
        partial_lon *= mu / r;

        let tmp1 = partial_r / r - z * partial_lat / (r.powi(2) * xy) ;
        let tmp2 = partial_lon / (xy * xy);        

        let ax = tmp1 * x - tmp2 * y;
        let ay = tmp1 * y + tmp2 * x;
        let az = partial_r * z / r + xy * partial_lat / r.powi(2);

        dbg!(&self.legendre.p);
        
        Vector3::new(ax, ay, az)
    }
}

#[cfg(test)]
mod tests {
    // use crate::egm96::EGM96;

    // use super::*;
    // use approx::assert_abs_diff_eq;
    // const TOL: f64 = 1e-10;

    // #[test]
    // fn test_spherical_harmonics() {
    //     let g = EGM96::new(3, 3);
    //     let mut spherical_harmonics = SphericalHarmonics::new(3, 3);
    //     let gradient = spherical_harmonics.calculate(&g.c, &g.s, 1e7, 0.0, 0.0, 1.0, 1.0);

    //     assert_abs_diff_eq!(gradient.x, 0.0, epsilon = TOL);
    //     assert_abs_diff_eq!(gradient.y, 0.0, epsilon = TOL);
    //     assert_abs_diff_eq!(gradient.z, 0.0, epsilon = TOL);
    // }
}
